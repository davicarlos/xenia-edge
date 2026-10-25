/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// The system HUD xam runs on every skeleton frame a title publishes
// (XamApp 0x2B002 → XamNuiHudInterpretFrame, 0x816ACC68 → 0x816AC688):
// - it picks the two players the tracker should follow and writes their
//   tracking IDs into the frame (0x8198B1E8);
// - while the system's Kinect UI is up, it scores the players
//   (XamNuiSkeletonScoreUpdate), hands a player who's at the edge, too close
//   or turned away back as position only, and watches for the wave that
//   engages a player (0x818092C8);
// - it watches for the Guide gesture, the left arm held out at 45 degrees for
//   three seconds (0x81808C60), and opens the Guide.

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/base/memory.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_nui.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/xbox.h"

DEFINE_bool(kinect_guide_gesture, true,
            "Open the Guide when a player holds the Kinect Guide gesture (the "
            "left arm out at 45 degrees for three seconds), as the console "
            "does in every Kinect title.",
            "HID");

namespace xe {
namespace kernel {
namespace xam {

X_RESULT xeXamShowGuide();
bool xeXamIsGuideOpen();

namespace {

constexpr uint32_t kJointHipCenter = 0;
constexpr uint32_t kJointShoulderCenter = 2;
constexpr uint32_t kJointHead = 3;
constexpr uint32_t kJointShoulderLeft = 4;
constexpr uint32_t kJointElbowLeft = 5;
constexpr uint32_t kJointHandLeft = 7;
constexpr uint32_t kJointShoulderRight = 8;
constexpr uint32_t kJointElbowRight = 9;
constexpr uint32_t kJointHandRight = 11;

struct Vec3 {
  float x = 0, y = 0, z = 0;
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
};
float Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float Length(const Vec3& v) { return std::sqrt(Dot(v, v)); }
bool Normalize(Vec3& v) {
  const float length = Length(v);
  if (length < 1e-6f) {
    return false;
  }
  v = v * (1.0f / length);
  return true;
}
constexpr float kDegrees = 57.2957795f;

// A skeleton of a big-endian frame in guest memory.
struct Skeleton {
  uint8_t* data;
  uint32_t state() const {
    return xe::load_and_swap<uint32_t>(data + kNuiSkeletonState);
  }
  void set_state(uint32_t state) {
    xe::store_and_swap<uint32_t>(data + kNuiSkeletonState, state);
  }
  uint32_t tracking_id() const {
    return xe::load_and_swap<uint32_t>(data + kNuiSkeletonTrackingId);
  }
  uint32_t enrollment() const {
    return xe::load_and_swap<uint32_t>(data + kNuiSkeletonEnrollment);
  }
  Vec3 position() const { return Load(data + kNuiSkeletonPosition); }
  Vec3 joint(uint32_t index) const {
    return Load(data + kNuiSkeletonJoints + index * 16);
  }
  uint32_t joint_state(uint32_t index) const {
    return xe::load_and_swap<uint32_t>(data + kNuiSkeletonJointStates +
                                       index * 4);
  }
  bool joint_tracked(uint32_t index) const {
    return joint_state(index) == kNuiStateTracked;
  }
  static Vec3 Load(const uint8_t* p) {
    return {xe::load_and_swap<float>(p), xe::load_and_swap<float>(p + 4),
            xe::load_and_swap<float>(p + 8)};
  }
};

Skeleton GetSkeleton(uint8_t* frame, uint32_t index) {
  return {frame + kNuiFrameSkeletons + index * kNuiSkeletonSize};
}

Vec3 FrameUp(const uint8_t* frame) {
  Vec3 up = Skeleton::Load(frame + kNuiFrameNormalToGravity);
  if (!Normalize(up)) {
    up = {0, 1, 0};
  }
  return up;
}

// --- Choosing the players (0x8198B1E8) ---------------------------------------

struct X_NUI_ENGAGEMENT {
  xe::be<uint32_t> engaged_tracking_id;
  xe::be<uint32_t> engaged_index;
  xe::be<uint32_t> engaged_user;
  xe::be<uint32_t> engaged_enrollment;
  xe::be<uint32_t> tracked_count;
  xe::be<uint32_t> tracked_indices[2];
};
static_assert_size(X_NUI_ENGAGEMENT, 0x1C);

// Picks the two people closest to the sensor across the floor, a tracked one
// gaining 0.3 m, and asks the tracker for them (frame+0xACC and +0xAD0).
void ChoosePlayers(uint8_t* frame, uint32_t engaged_id,
                   X_NUI_ENGAGEMENT* out) {
  if (out) {
    std::memset(out, 0, sizeof(*out));
    out->engaged_index = 0xFFFFFFFF;
    out->engaged_user = 0xFE;
    out->engaged_enrollment = 0xFFFFFFFE;
  }
  uint32_t chosen_ids[2] = {};
  float chosen_scores[2] = {FLT_MAX, FLT_MAX};
  uint32_t tracked_count = 0;
  for (uint32_t i = 0; i < kNuiSkeletonCount; ++i) {
    Skeleton skeleton = GetSkeleton(frame, i);
    const uint32_t state = skeleton.state();
    if (!state) {
      continue;
    }
    if (state == kNuiStateTracked && tracked_count < 2 && out) {
      out->tracked_indices[tracked_count] = i;
    }
    tracked_count += state == kNuiStateTracked;
    const uint32_t id = skeleton.tracking_id();
    if (out && id == engaged_id) {
      out->engaged_tracking_id = id;
      out->engaged_index = i;
      out->engaged_user = xe::load_and_swap<uint32_t>(skeleton.data +
                                                      kNuiSkeletonUserIndex);
      out->engaged_enrollment = skeleton.enrollment();
    }
    const Vec3 position = skeleton.position();
    float score = std::sqrt(position.x * position.x + position.z * position.z);
    if (state == kNuiStateTracked) {
      score -= 0.3f;
    }
    score = std::max(score, 0.1f);
    uint32_t candidate = id;
    for (uint32_t slot = 0; slot < 2; ++slot) {
      if (score < chosen_scores[slot]) {
        std::swap(score, chosen_scores[slot]);
        std::swap(candidate, chosen_ids[slot]);
      }
    }
  }
  if (out) {
    out->tracked_count = std::min(tracked_count, 2u);
  }
  xe::store_and_swap<uint32_t>(frame + kNuiFrameTrackedIds, chosen_ids[0]);
  xe::store_and_swap<uint32_t>(frame + kNuiFrameTrackedIds + 4,
                               chosen_ids[1]);
}

// The first chosen player who is tracked (0x8198B170), or -1.
int32_t BestSkeletonIndex(uint8_t* frame) {
  for (uint32_t slot = 0; slot < 2; ++slot) {
    const uint32_t id =
        xe::load_and_swap<uint32_t>(frame + kNuiFrameTrackedIds + slot * 4);
    if (!id) {
      continue;
    }
    for (uint32_t i = 0; i < kNuiSkeletonCount; ++i) {
      Skeleton skeleton = GetSkeleton(frame, i);
      if (skeleton.tracking_id() == id &&
          skeleton.state() == kNuiStateTracked) {
        return int32_t(i);
      }
    }
  }
  return -1;
}

// --- Scoring the players (XamNuiSkeletonScoreUpdate, 0x8198B628) -------------

struct X_NUI_SKELETON_SCORE {
  xe::be<uint32_t> tracking_state;
  // The tracking state, or position only when any warning is set.
  xe::be<uint32_t> state;
  xe::be<float> shoulders_lost;
  xe::be<uint32_t> upper_body_lost;
  // -1 at the image's left edge to 1 at its right, and near either.
  xe::be<float> horizontal_position;
  xe::be<uint32_t> near_side_edge;
  xe::be<float> distance;
  xe::be<uint32_t> too_close;
  // Degrees the shoulders turn from facing the sensor, and turned away.
  xe::be<float> turn_degrees;
  xe::be<uint32_t> turned;
  xe::be<float> hand_tracked;
  xe::be<uint32_t> no_hands;
  xe::be<uint32_t> tracking_id;
  xe::be<uint32_t> skeleton_index;
  xe::be<uint32_t> is_new;
};
static_assert_size(X_NUI_SKELETON_SCORE, 0x3C);

struct X_NUI_SKELETON_SCORES {
  X_NUI_SKELETON_SCORE players[2];
  xe::be<uint32_t> tracked_count;
};
static_assert_size(X_NUI_SKELETON_SCORES, 0x7C);

// 0x8198B390.
void ScorePlayer(Skeleton skeleton, const Vec3& up, X_NUI_SKELETON_SCORE& out) {
  const uint32_t tracking_state = skeleton.state();
  out.tracking_state = tracking_state;
  const uint32_t shoulders_lost =
      !skeleton.joint_tracked(kJointShoulderLeft) +
      !skeleton.joint_tracked(kJointShoulderRight);
  out.shoulders_lost = float(shoulders_lost);
  out.upper_body_lost =
      shoulders_lost == 2 && !skeleton.joint_tracked(kJointHead);
  // Projected into the 320x240 depth image (0x8198B0F0).
  const Vec3 position = skeleton.position();
  const float px = position.z != 0 ? position.x * 285.63f / position.z + 160.0f
                                   : 0.0f;
  const float horizontal = (160.0f - px) / 160.0f;
  out.horizontal_position = horizontal;
  out.near_side_edge = std::abs(horizontal) >= 0.7f;
  out.distance = position.z;
  out.too_close = position.z <= 1.0f;
  // From the shoulder nearer the camera to the shoulders' center, across the
  // floor, against the image's horizontal.
  const Vec3 left = skeleton.joint(kJointShoulderLeft);
  const Vec3 right = skeleton.joint(kJointShoulderRight);
  Vec3 shoulder = (left.z < right.z ? left : right);
  Vec3 across = skeleton.joint(kJointShoulderCenter) - shoulder;
  across = across - up * Dot(across, up);
  float turn = 0.0f;
  if (Normalize(across)) {
    turn = std::acos(std::clamp(across.x, -1.0f, 1.0f)) * kDegrees;
  }
  out.turn_degrees = turn;
  // Measured from either shoulder, so either way round.
  out.turned = std::min(turn, 180.0f - turn) >= 45.0f;
  const bool hand = skeleton.joint_tracked(kJointHandLeft) ||
                    skeleton.joint_tracked(kJointHandRight);
  out.hand_tracked = hand ? 1.0f : 0.0f;
  out.no_hands = !hand;
  const bool warning = out.upper_body_lost || out.near_side_edge ||
                       out.too_close || out.turned || out.no_hands;
  out.state = warning && tracking_state ? kNuiStatePositionOnly
                                        : tracking_state;
}

void UpdateScores(uint8_t* frame, X_NUI_SKELETON_SCORES& scores) {
  const Vec3 up = FrameUp(frame);
  uint32_t tracked[2];
  uint32_t count = 0;
  for (uint32_t i = 0; i < kNuiSkeletonCount && count < 2; ++i) {
    if (GetSkeleton(frame, i).state() == kNuiStateTracked) {
      tracked[count++] = i;
    }
  }
  // With one player left, in the second record, it moves to the first.
  if (count == 1 && GetSkeleton(frame, tracked[0]).tracking_id() ==
                        uint32_t(scores.players[1].tracking_id)) {
    scores.players[0] = scores.players[1];
    std::memset(&scores.players[1], 0, sizeof(scores.players[1]));
  }
  bool matched[2] = {};
  uint32_t record_of[2] = {};
  for (uint32_t p = 0; p < count; ++p) {
    const uint32_t id = GetSkeleton(frame, tracked[p]).tracking_id();
    record_of[p] = 2;
    for (uint32_t r = 0; r < 2; ++r) {
      if (!matched[r] && uint32_t(scores.players[r].tracking_id) == id) {
        record_of[p] = r;
        matched[r] = true;
        break;
      }
    }
  }
  for (uint32_t p = 0; p < count; ++p) {
    bool is_new = false;
    if (record_of[p] == 2) {
      for (uint32_t r = 0; r < 2; ++r) {
        if (!matched[r]) {
          record_of[p] = r;
          matched[r] = true;
          break;
        }
      }
      is_new = true;
    }
    X_NUI_SKELETON_SCORE& record = scores.players[record_of[p]];
    if (is_new) {
      std::memset(&record, 0, sizeof(record));
    }
    Skeleton skeleton = GetSkeleton(frame, tracked[p]);
    ScorePlayer(skeleton, up, record);
    record.tracking_id = skeleton.tracking_id();
    record.skeleton_index = tracked[p];
    record.is_new = is_new;
  }
  for (uint32_t r = 0; r < 2; ++r) {
    if (!matched[r]) {
      std::memset(&scores.players[r], 0, sizeof(scores.players[r]));
    }
  }
  scores.tracked_count = count;
}

// --- The body's frame of reference (0x818098E0) ------------------------------

// Up is the normal to gravity; out runs along the shoulder line from the
// right shoulder to the left; forward is out × up.
struct BodyFrame {
  Vec3 up, out, forward;
};

bool MakeBodyFrame(Skeleton skeleton, const Vec3& up, const Vec3& left_shoulder,
                   const Vec3& right_shoulder, BodyFrame& body) {
  body.up = up;
  Vec3 out = left_shoulder - right_shoulder;
  out = out - up * Dot(out, up);
  if (!Normalize(out)) {
    return false;
  }
  body.out = out;
  body.forward = Cross(out, up);
  return true;
}

// Degrees between two arm segments: 0 for a straight arm.
float Bend(const Vec3& shoulder, const Vec3& elbow, const Vec3& hand) {
  Vec3 upper = elbow - shoulder, lower = hand - elbow;
  if (!Normalize(upper) || !Normalize(lower)) {
    return 0.0f;
  }
  return std::acos(std::clamp(Dot(upper, lower), -1.0f, 1.0f)) * kDegrees;
}

// --- The engagement wave (0x818092C8 → 0x8180AAB8, per hand) -----------------

// 0x8180A388.
constexpr uint32_t kWaveHalfWaves = 4;
constexpr uint32_t kWaveDirectionFrames = 3;
constexpr float kWaveRaisedMs = 1000.0f;
constexpr float kWaveGapMs = 1000.0f;
constexpr float kWaveSpeed = 0.7f;
constexpr float kWaveSpeedFewFrames = 1.5f;
constexpr float kWaveRhythm = 0.6f;

struct WaveHand {
  uint32_t tracking_id = 0;
  float raised_ms = 0;
  float last_x = 0;
  bool has_last = false;
  int direction = 0;
  uint32_t direction_frames = 0;
  uint32_t half_waves = 0;
  float since_half_wave_ms = 0;
  float average_period_ms = 0;
};

// One frame of a hand; true when the wave completes.
bool UpdateWave(WaveHand& wave, Skeleton skeleton, const BodyFrame& body,
                const X_NUI_SKELETON_SCORE* score, bool right, float dt_ms) {
  const uint32_t shoulder_joint = right ? kJointShoulderRight
                                        : kJointShoulderLeft;
  const uint32_t elbow_joint = right ? kJointElbowRight : kJointElbowLeft;
  const uint32_t hand_joint = right ? kJointHandRight : kJointHandLeft;
  const Vec3 hand = skeleton.joint(hand_joint);
  const Vec3 elbow = skeleton.joint(elbow_joint);
  const Vec3 shoulder = skeleton.joint(shoulder_joint);
  // Raised: above the elbow and no more than 6 cm farther from the camera
  // than the shoulder, facing the sensor (0x8180A788).
  bool raised = skeleton.joint_tracked(hand_joint) &&
                Dot(hand - elbow, body.up) > 0 &&
                hand.z <= shoulder.z + 0.06f;
  if (raised && score) {
    const float turn = score->turn_degrees;
    raised = std::min(turn, 180.0f - turn) < 45.0f;
  }
  if (!raised) {
    wave = WaveHand();
    wave.tracking_id = skeleton.tracking_id();
    return false;
  }
  wave.raised_ms += dt_ms;
  const float x = Dot(hand, body.out);
  if (!wave.has_last || dt_ms <= 0) {
    wave.last_x = x;
    wave.has_last = true;
    return false;
  }
  const float speed = (x - wave.last_x) / (dt_ms * 0.001f);
  wave.last_x = x;
  wave.since_half_wave_ms += dt_ms;
  if (wave.since_half_wave_ms > kWaveGapMs) {
    wave.half_waves = 0;
    wave.average_period_ms = 0;
  }
  if (wave.raised_ms < kWaveRaisedMs || speed == 0) {
    return false;
  }
  const int direction = speed > 0 ? 3 : 2;
  if (direction != wave.direction) {
    // A half wave: the hand turns after enough frames fast enough.
    const float needed = wave.direction_frames >= kWaveDirectionFrames
                             ? kWaveSpeed
                             : kWaveSpeedFewFrames;
    if (wave.direction && std::abs(speed) >= needed) {
      const float period = wave.since_half_wave_ms;
      if (wave.half_waves && wave.average_period_ms > 0 &&
          std::abs(period - wave.average_period_ms) >
              kWaveRhythm * wave.average_period_ms) {
        wave.half_waves = 0;
        wave.average_period_ms = 0;
      }
      wave.average_period_ms =
          wave.half_waves ? (wave.average_period_ms * wave.half_waves +
                             period) /
                                (wave.half_waves + 1)
                          : period;
      ++wave.half_waves;
      wave.since_half_wave_ms = 0;
    }
    wave.direction = direction;
    wave.direction_frames = 1;
  } else {
    ++wave.direction_frames;
  }
  return wave.half_waves >= kWaveHalfWaves;
}

// --- The Guide gesture (0x81808C60 → 0x8180A0B0, 0x81809AF0) -----------------

constexpr float kGuideHoldMs = 3000.0f;
constexpr float kGuideCooldownMs = 1500.0f;
constexpr float kGuideHandSmoothing = 0.1f;
constexpr float kGuideShoulderSmoothing = 0.05f;

struct GuidePose {
  uint32_t tracking_id = 0;
  bool smoothed = false;
  Vec3 left_hand, right_hand, left_shoulder, right_shoulder;
  float held_ms = 0;
  float cooldown_ms = 0;
};

// The arm's angles, 0 hanging straight down: out to its side across the
// body's front, and forward or back along its side.
void ArmAngles(const Vec3& shoulder, const Vec3& hand, const BodyFrame& body,
               float side_sign, float& sideways, float& forward) {
  const Vec3 arm = hand - shoulder;
  const float down = -Dot(arm, body.up);
  sideways = std::atan2(side_sign * Dot(arm, body.out), down) * kDegrees;
  forward = std::atan2(Dot(arm, body.forward), down) * kDegrees;
}

// With flag 4 or 8 of the frame (+0xAB8) the pose is relaxed: the left arm up
// to 95 degrees out and 40 degrees bent, and the right arm and the trunk
// aren't checked.
bool GuidePoseHeld(GuidePose& pose, Skeleton skeleton, const Vec3& up,
                   bool relaxed) {
  if (!skeleton.joint_tracked(kJointShoulderLeft) ||
      !skeleton.joint_tracked(kJointShoulderRight)) {
    return false;
  }
  auto smooth = [&](Vec3& value, const Vec3& sample, float factor) {
    value = pose.smoothed ? value + (sample - value) * factor : sample;
  };
  smooth(pose.left_hand, skeleton.joint(kJointHandLeft), kGuideHandSmoothing);
  smooth(pose.right_hand, skeleton.joint(kJointHandRight),
         kGuideHandSmoothing);
  smooth(pose.left_shoulder, skeleton.joint(kJointShoulderLeft),
         kGuideShoulderSmoothing);
  smooth(pose.right_shoulder, skeleton.joint(kJointShoulderRight),
         kGuideShoulderSmoothing);
  pose.smoothed = true;
  BodyFrame body;
  if (!MakeBodyFrame(skeleton, up, pose.left_shoulder, pose.right_shoulder,
                     body)) {
    return false;
  }
  // The left arm 205-245 degrees across (25-65 out from hanging), 250-290
  // along (within 20 of hanging), the elbow straighter than 35 degrees.
  float left_out, left_forward;
  ArmAngles(pose.left_shoulder, pose.left_hand, body, 1.0f, left_out,
            left_forward);
  const float left_bend =
      Bend(skeleton.joint(kJointShoulderLeft), skeleton.joint(kJointElbowLeft),
           skeleton.joint(kJointHandLeft));
  const bool left = relaxed ? left_out > 25.0f && left_out < 95.0f &&
                                  std::abs(left_forward) < 25.0f &&
                                  left_bend < 40.0f
                            : left_out > 25.0f && left_out < 65.0f &&
                                  std::abs(left_forward) < 20.0f &&
                                  left_bend < 35.0f;
  if (!left || relaxed) {
    return left;
  }
  // The right arm hanging within 20 degrees and bent under 110, unless its
  // hand or shoulder isn't tracked.
  if (skeleton.joint_tracked(kJointHandRight) &&
      skeleton.joint_tracked(kJointShoulderRight)) {
    float right_out, right_forward;
    ArmAngles(pose.right_shoulder, pose.right_hand, body, -1.0f, right_out,
              right_forward);
    const float right_bend = Bend(skeleton.joint(kJointShoulderRight),
                                  skeleton.joint(kJointElbowRight),
                                  skeleton.joint(kJointHandRight));
    if (std::abs(right_out) >= 20.0f || right_bend >= 110.0f) {
      return false;
    }
  }
  // The trunk within 9 degrees of upright, forward or back.
  const Vec3 spine = skeleton.joint(kJointShoulderCenter) -
                     skeleton.joint(kJointHipCenter);
  const float lean =
      std::atan2(Dot(spine, body.forward), Dot(spine, body.up)) * kDegrees;
  return std::abs(lean) < 9.0f;
}

// --- The HUD -----------------------------------------------------------------

struct HudState {
  std::mutex mutex;
  std::chrono::steady_clock::time_point last_frame;
  bool has_last_frame = false;
  X_NUI_SKELETON_SCORES scores = {};
  std::array<std::array<WaveHand, 2>, 2> waves;
  std::array<GuidePose, kNuiSkeletonCount> guide;
  // The last frame, for XamNuiHudGetEngagedEnrollmentIndex.
  std::array<uint8_t, kNuiFrameSize> frame = {};
  bool has_frame = false;
  uint32_t input_filter = 0;
};

HudState& GetHud() {
  static HudState hud;
  return hud;
}

// Returns whether the Guide gesture completed.
bool InterpretFrame(HudState& hud, uint8_t* frame, float dt_ms) {
  NUIController* nui = kernel_state()->nui();
  const uint32_t init_flags = nui->GetInitFlags();
  if (init_flags & 0x200) {
    xe::store_and_swap<uint32_t>(
        frame + kNuiFrameFlags,
        xe::load_and_swap<uint32_t>(frame + kNuiFrameFlags) | 4);
  }
  const Vec3 up = FrameUp(frame);
  ChoosePlayers(frame, nui->GetEngagedTrackingId(), nullptr);

  if (kernel_state()->xam_state()->IsUIActive()) {
    UpdateScores(frame, hud.scores);
    if (!(init_flags & 0x200)) {
      // A player with a warning reaches the title as position only.
      for (const auto& record : hud.scores.players) {
        if (!record.tracking_id) {
          continue;
        }
        Skeleton skeleton = GetSkeleton(frame, record.skeleton_index);
        skeleton.set_state(record.state);
      }
    }
    uint32_t engaged = nui->GetEngagedTrackingId();
    bool engaged_present = false;
    for (uint32_t i = 0; i < kNuiSkeletonCount; ++i) {
      Skeleton skeleton = GetSkeleton(frame, i);
      engaged_present |= engaged && skeleton.tracking_id() == engaged &&
                         skeleton.state() == kNuiStateTracked;
    }
    if (engaged && !engaged_present) {
      nui->SetEngagedTrackingId(0);
      engaged = 0;
    }
    // The other players' waves; who completes one is engaged.
    for (uint32_t p = 0; p < 2; ++p) {
      const auto& record = hud.scores.players[p];
      if (!record.tracking_id || record.tracking_id == engaged) {
        continue;
      }
      Skeleton skeleton = GetSkeleton(frame, record.skeleton_index);
      BodyFrame body;
      if (!MakeBodyFrame(skeleton, up, skeleton.joint(kJointShoulderLeft),
                         skeleton.joint(kJointShoulderRight), body)) {
        continue;
      }
      for (uint32_t h = 0; h < 2; ++h) {
        WaveHand& wave = hud.waves[p][h];
        if (wave.tracking_id != record.tracking_id) {
          wave = WaveHand();
          wave.tracking_id = record.tracking_id;
        }
        if (UpdateWave(wave, skeleton, body, &record, h == 1, dt_ms)) {
          nui->SetEngagedTrackingId(record.tracking_id);
          XELOGI("Kinect: player {:08X} engaged by waving",
                 uint32_t(record.tracking_id));
          hud.waves[p] = {};
          break;
        }
      }
    }
  }
  std::memcpy(hud.frame.data(), frame, kNuiFrameSize);
  hud.has_frame = true;

  // The Guide gesture.
  const bool relaxed =
      xe::load_and_swap<uint32_t>(frame + kNuiFrameFlags) & (4 | 8);
  bool open_guide = false;
  for (uint32_t i = 0; i < kNuiSkeletonCount; ++i) {
    Skeleton skeleton = GetSkeleton(frame, i);
    GuidePose& pose = hud.guide[i];
    pose.cooldown_ms = std::max(pose.cooldown_ms - dt_ms, 0.0f);
    if (pose.tracking_id != skeleton.tracking_id()) {
      pose = GuidePose();
      pose.tracking_id = skeleton.tracking_id();
    }
    const bool held = cvars::kinect_guide_gesture && !xeXamIsGuideOpen() &&
                      skeleton.state() == kNuiStateTracked &&
                      GuidePoseHeld(pose, skeleton, up, relaxed);
    if (held && pose.cooldown_ms == 0.0f) {
      pose.held_ms += dt_ms;
    } else {
      pose.held_ms = 0.0f;
    }
    if (pose.held_ms >= kGuideHoldMs) {
      open_guide = true;
      XELOGI("Kinect: Guide gesture by player {:08X}", pose.tracking_id);
    }
  }
  if (open_guide) {
    // Everyone waits before the gesture counts again.
    for (auto& pose : hud.guide) {
      pose.held_ms = 0.0f;
      pose.cooldown_ms = kGuideCooldownMs;
    }
  }
  return open_guide;
}

}  // namespace

X_HRESULT xeXamNuiHudInterpretFrame(uint8_t* frame) {
  NUIController* nui = kernel_state()->nui();
  X_HRESULT result = X_E_FAIL;
  bool open_guide = false;
  if (frame && nui->IsHudEnabled() && nui->IsDeviceReady()) {
    HudState& hud = GetHud();
    std::lock_guard<std::mutex> lock(hud.mutex);
    const auto now = std::chrono::steady_clock::now();
    float dt_ms = 0.0f;
    if (hud.has_last_frame) {
      // A pause counts as one frame.
      dt_ms = std::min(
          std::chrono::duration<float, std::milli>(now - hud.last_frame)
              .count(),
          100.0f);
    }
    hud.last_frame = now;
    hud.has_last_frame = true;
    open_guide = InterpretFrame(hud, frame, dt_ms);
    result = hud.input_filter ? X_HRESULT(0x8301000B) : X_E_SUCCESS;
  }
  // Opened unlocked: the Guide pauses the title, which may wait on the HUD.
  if (open_guide) {
    xeXamShowGuide();
  }
  // The system has the player's input while its UI is up.
  if (kernel_state()->xam_state()->IsUIActive()) {
    result = 0x8301000B;
  }
  return result;
}

dword_result_t XamNuiHudInterpretFrame_entry(lpvoid_t frame) {
  return xeXamNuiHudInterpretFrame(frame ? frame.as<uint8_t*>() : nullptr);
}
DECLARE_XAM_EXPORT1(XamNuiHudInterpretFrame, kNone, kImplemented);

// The HUD is there while a title runs it (0x2B003 without bit 31) with a
// sensor (0x816AC4B8).
dword_result_t XamNuiHudIsEnabled_entry() {
  NUIController* nui = kernel_state()->nui();
  return nui->IsHudEnabled() && nui->IsDeviceReady() ? 1 : 0;
}
DECLARE_XAM_EXPORT1(XamNuiHudIsEnabled, kNone, kImplemented);

dword_result_t XamNuiHudGetInitializeFlags_entry() {
  return kernel_state()->nui()->GetInitFlags();
}
DECLARE_XAM_EXPORT1(XamNuiHudGetInitializeFlags, kNone, kImplemented);

void XamNuiHudGetVersions_entry(lpqword_t xam_version,
                                lpqword_t xtl_version) {
  NUIController* nui = kernel_state()->nui();
  if (xam_version) {
    *xam_version = nui->GetNUIVersion(0);
  }
  if (xtl_version) {
    *xtl_version = nui->GetNUIVersion(1);
  }
}
DECLARE_XAM_EXPORT1(XamNuiHudGetVersions, kNone, kImplemented);

dword_result_t XamNuiHudGetEngagedTrackingID_entry() {
  return kernel_state()->nui()->GetEngagedTrackingId();
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedTrackingID, kNone, kImplemented);

// Without the HUD xam refuses (0x816AC428).
dword_result_t XamNuiHudSetEngagedTrackingID_entry(dword_t tracking_id) {
  if (!XamNuiHudIsEnabled_entry()) {
    return X_E_FAIL;
  }
  kernel_state()->nui()->SetEngagedTrackingId(tracking_id);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudSetEngagedTrackingID, kNone, kImplemented);

uint32_t xeXamNuiHudEnrollmentOf(uint32_t tracking_id) {
  HudState& hud = GetHud();
  std::lock_guard<std::mutex> lock(hud.mutex);
  if (hud.has_frame && tracking_id) {
    for (uint32_t i = 0; i < kNuiSkeletonCount; ++i) {
      Skeleton skeleton = GetSkeleton(hud.frame.data(), i);
      if (skeleton.state() && skeleton.tracking_id() == tracking_id) {
        return skeleton.enrollment();
      }
    }
  }
  return 0xFFFFFFFE;
}

// The engaged player's enrollment in the HUD's copy of the last frame, or -2
// (0x816ACCE8).
dword_result_t XamNuiHudGetEngagedEnrollmentIndex_entry() {
  return xeXamNuiHudEnrollmentOf(kernel_state()->nui()->GetEngagedTrackingId());
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedEnrollmentIndex, kNone, kImplemented);

// While on, InterpretFrame tells the title not to use its frames: the system
// has the input (0x816C7EB8).
dword_result_t XamNuiHudEnableInputFilter_entry(dword_t enable) {
  if (!XamNuiHudIsEnabled_entry()) {
    return X_E_SUCCESS;
  }
  HudState& hud = GetHud();
  std::lock_guard<std::mutex> lock(hud.mutex);
  hud.input_filter = enable;
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudEnableInputFilter, kNone, kImplemented);

// The same choice of players, for titles that run it themselves.
void XamNuiPlayerEngagementUpdate_entry(lpvoid_t frame, dword_t engaged_id,
                                        pointer_t<X_NUI_ENGAGEMENT> out) {
  if (frame) {
    ChoosePlayers(frame.as<uint8_t*>(), engaged_id, out ? &*out : nullptr);
  }
}
DECLARE_XAM_EXPORT1(XamNuiPlayerEngagementUpdate, kNone, kImplemented);

dword_result_t XamNuiSkeletonGetBestSkeletonIndex_entry(lpvoid_t frame) {
  return frame ? uint32_t(BestSkeletonIndex(frame.as<uint8_t*>()))
               : 0xFFFFFFFF;
}
DECLARE_XAM_EXPORT1(XamNuiSkeletonGetBestSkeletonIndex, kNone, kImplemented);

void XamNuiSkeletonScoreUpdate_entry(lpvoid_t frame,
                                     pointer_t<X_NUI_SKELETON_SCORES> scores) {
  if (frame && scores) {
    UpdateScores(frame.as<uint8_t*>(), *scores);
  }
}
DECLARE_XAM_EXPORT1(XamNuiSkeletonScoreUpdate, kNone, kImplemented);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUIHud);
