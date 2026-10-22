/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2021 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/xam/apps/xam_app.h"

#include "xenia/base/logging.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/xam/xam_content_device.h"
#include "xenia/kernel/xenumerator.h"

/* Notes:
   - Messages ids that start with 0x00021xxx are UI calls
   - Messages ids that start with 0x00023xxx are used for the user profile
   - Messages ids that start with 0x0002Bxxx are used by the Kinect device
   usually for camera related functions
   - Messages ids that start with 0x0002Cxxx are used by the XamNuiIdentity
   functions
*/

namespace xe {
namespace kernel {
namespace xam {
namespace apps {

XamApp::XamApp(KernelState* kernel_state) : App(kernel_state, 0xFE) {}

X_HRESULT XamApp::DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                      uint32_t buffer_length) {
  // NOTE: buffer_length may be zero or valid.
  auto buffer = memory_->TranslateVirtual(buffer_ptr);
  switch (message) {
    case 0x0002000E: {
      X_ENUMERATE_PARAM* data_ptr =
          reinterpret_cast<X_ENUMERATE_PARAM*>(buffer);

      XELOGD(
          "XEnumerateCrossTitle({:04X}, {:04X}, {:04X}, {:04X}, {}, {}, "
          "{:04X})",
          data_ptr->user_index.get(), data_ptr->flags.get(),
          data_ptr->private_enum_structure_ptr.get(),
          data_ptr->buffer_ptr.get(), data_ptr->buffer_size.get(),
          data_ptr->items_requested.get(), data_ptr->items_returned_ptr.get());

      if (!data_ptr->buffer_ptr || !data_ptr->private_enum_structure_ptr) {
        return X_E_INVALIDARG;
      }

      auto enum_struct =
          memory_->TranslateVirtual<X_KENUMERATOR_CONTENT_AGGREGATE*>(
              data_ptr->private_enum_structure_ptr);

      auto e = kernel_state_->object_table()->LookupObject<XEnumerator>(
          enum_struct->handle);

      if (!e) {
        return X_E_INVALIDARG;
      }

      assert_true(enum_struct->magic == kXObjSignature);

      XCONTENT_CROSS_TITLE_DATA cross_title_data = {};
      uint8_t* cross_title_data_ptr =
          reinterpret_cast<uint8_t*>(&cross_title_data);

      uint32_t item_count = 0;
      X_RESULT result = e->WriteItems(cross_title_data_ptr,
                                      data_ptr->buffer_size, &item_count);

      XCONTENT_DATA_INTERNAL* content_data_ptr =
          memory_->TranslateVirtual<XCONTENT_DATA_INTERNAL*>(
              data_ptr->buffer_ptr);

      assert_true(data_ptr->buffer_size == sizeof(XCONTENT_DATA_INTERNAL));

      std::memset(content_data_ptr, 0, data_ptr->buffer_size);

      if (!result) {
        content_data_ptr->device_id = cross_title_data.content_data.device_id;
        content_data_ptr->content_type =
            cross_title_data.content_data.content_type;
        content_data_ptr->set_display_name(
            cross_title_data.content_data.display_name());
        content_data_ptr->set_file_name(
            cross_title_data.content_data.file_name());
        content_data_ptr->padding[0] = content_data_ptr->padding[1] = 0;
        content_data_ptr->title_id = cross_title_data.title_id;
      }

      result = X_HRESULT_FROM_WIN32(result);

      xe::be<uint32_t>* items_returned_ptr =
          memory_->TranslateVirtual<xe::be<uint32_t>*>(
              data_ptr->items_returned_ptr);

      *items_returned_ptr = item_count;

      return result;
    }
    case 0x00020021: {
      struct XContentQueryVolumeDeviceType {
        char root_name[64];
        xe::be<uint32_t> is_title_process;
        xe::be<DeviceType> device_type_ptr;
        xe::be<uint32_t> overlapped_ptr;
      }* data = reinterpret_cast<XContentQueryVolumeDeviceType*>(buffer);
      assert_true(buffer_length == sizeof(XContentQueryVolumeDeviceType));

      std::string target;
      if (!kernel_state_->file_system()->FindSymbolicLink(
              std::string(data->root_name) + ':', target)) {
        return X_E_INVALIDARG;
      }

      // Content packages are mounted from the HDD content store.
      const bool is_content_package = target.starts_with("\\Device\\Content\\");

      // Only apply this check to XContent packages
      if (!is_content_package && !target.starts_with("\\Device\\Package_")) {
        return X_E_INVALIDARG;
      }

      xe::be<DeviceType>* device_type_ptr =
          memory_->TranslateVirtual<xe::be<DeviceType>*>(
              static_cast<uint32_t>(data->device_type_ptr.get()));

      if (is_content_package) {
        *device_type_ptr = DeviceType::HDD;
      } else {
        switch (kernel_state_->deployment_type_) {
          case XDeploymentType::kDownload:
          case XDeploymentType::kInstalledToHDD: {
            *device_type_ptr = DeviceType::HDD;
          } break;
          case XDeploymentType::kOpticalDisc: {
            *device_type_ptr = DeviceType::ODD;
          } break;
          default: {
            *device_type_ptr = DeviceType::Invalid;
          } break;
        }
      }

      XELOGD("XContentQueryVolumeDeviceType('{}', {:08X}, {:08X}, {:08X})",
             data->root_name,
             static_cast<uint32_t>(data->is_title_process.get()),
             static_cast<uint32_t>(data->device_type_ptr.get()),
             static_cast<uint32_t>(data->overlapped_ptr.get()));

      return X_E_SUCCESS;
    }
    case 0x00021012: {
      uint32_t enabled = xe::load_and_swap<uint32_t>(buffer);
      XELOGD("XEnableGuestSignin: {}", enabled ? "true" : "false");
      return X_E_SUCCESS;
    }
    case 0x00022005: {
      struct XTITLE_GET_DEPLOYMENT_TYPE {
        xe::be<uint32_t> deployment_type_ptr;
        xe::be<uint32_t> overlapped_ptr;
      }* data = reinterpret_cast<XTITLE_GET_DEPLOYMENT_TYPE*>(buffer);
      assert_true(!buffer_length ||
                  buffer_length == sizeof(XTITLE_GET_DEPLOYMENT_TYPE));
      auto deployment_type =
          memory_->TranslateVirtual<uint32_t*>(data->deployment_type_ptr);
      *deployment_type = static_cast<uint32_t>(kernel_state_->deployment_type_);
      XELOGD("XTitleGetDeploymentType({:08X}, {:08X}",
             data->deployment_type_ptr.get(), data->overlapped_ptr.get());
      return X_E_SUCCESS;
    }
    case 0x0002B001: {
      // Games used in:
      // 42560819, 373307D9
      uint32_t ms_delay = xe::load_and_swap<uint32_t>(buffer);

      XELOGD("XNuiDelayUI unimplemented({:08X})", ms_delay);
      return X_E_SUCCESS;
    }
    case 0x0002B003: {
      // Games used in:
      // 4D5309C9
      // It only receives buffer
      X_NUI_TITLE_INITIALIZE_DATA* args =
          reinterpret_cast<X_NUI_TITLE_INITIALIZE_DATA*>(buffer);
      XELOGD("XNuiTitleInitialize({:016X}, {:016X}, flags: 0x{:08X}), Stubbed",
             args->xam_version.get(), args->xtl_version.get(),
             args->flags.get());
      X_HRESULT result = X_E_FAIL;

      if (args->xam_version == 5) {
        kernel_state_->nui()->SetNUIVersion(uint64_t(args->xam_version), 0);
        kernel_state_->nui()->SetNUIVersion(uint64_t(args->xtl_version), 1);
        kernel_state_->nui()->SetInitFlags(uint32_t(args->flags));
        if ((args->flags & 0x80000000) == 0) {
          XELOGD("Todo: write data to two pointers");
          result = X_E_SUCCESS;
        }
      }
      if ((args->flags & 0x40000000) == 0) {
        XELOGD(
            "Todo: call XamNuiGetDeviceStatus and if status does not have flag "
            "1 set but has flags 4 and 0x40 call XamShowNuiTroubleshooterUI");
      }

      return result;
    }
    case 0x0002B004: {
      // Games used in:
      // Emerald Dashboard, Forza Horizon 2
      // Called with no buffer

      kernel_state_->nui()->SetNUIVersion(0, 0);
      kernel_state_->nui()->SetNUIVersion(0, 1);
      kernel_state_->nui()->SetInitFlags(0);
      kernel_state_->nui()->SetTiltCallback(0);

      XELOGD("XNuiTitleShutdown stubbed");
      return X_E_SUCCESS;
    }
    case 0x0002B002: {
      // The NUI runtime publishes each skeleton frame it fits to xam's HUD:
      // 0xAE0 bytes with the runtime version, the floor plane, six
      // NUI_SKELETON_DATA and the frame's timestamp. Nothing consumes it yet.
      return X_E_SUCCESS;
    }
    case 0x0002A004:
    case 0x0002A005:
    case 0x0002A006:
    case 0x0002A007:
    case 0x0002A008: {
      // Identity and biometric profile data requests, 24 bytes each, sent
      // with XMsgStartIORequest and no overlapped during the runtime's
      // identity initialization (0x2A005, 0x2A006) and for each signed-in
      // profile (0x2A008 asks whether xam holds the profile's biometric data,
      // 0x2A007 hands xam the runtime's copy). What xam does with them isn't
      // known; the runtime only needs them to succeed, and a stamp left at 0
      // by 0x2A008 tells it xam holds nothing.
      XELOGD("XamApp: NUI identity I/O request {:08X} ({} bytes)", message,
             buffer_length);
      return X_E_SUCCESS;
    }
    case 0x0002C00C: {
      // Registers one identity feature (classifier): {version, flags,
      // reserved, unk, name pointer, name length, ...}. The runtime registers
      // about twenty, named like NuiV1FaceHiRes, during its identity
      // initialization and needs each to succeed.
      if (!buffer_ptr) {
        return X_E_INVALIDARG;
      }
      const uint32_t name_ptr = xe::load_and_swap<uint32_t>(buffer + 16);
      const uint32_t name_length = xe::load_and_swap<uint32_t>(buffer + 20);
      std::string_view name;
      if (name_ptr && name_length && name_length < 256) {
        name = std::string_view(memory_->TranslateVirtual<const char*>(name_ptr),
                                name_length);
      }
      XELOGD("XamApp: NUI identity feature registered: {} (flags {:08X})", name,
             xe::load_and_swap<uint32_t>(buffer + 4));
      return X_E_SUCCESS;
    }
    case 0x0002C009: {
      // Registers (non-zero) or removes (zero) a Kinect identity callback.
      // TODO(knuckleslee): Invoke the callback for identity notifications.
      if (!buffer_ptr) {
        return X_E_INVALIDARG;
      }
      XELOGD("XamApp: Kinect identity callback {:08X}",
             xe::load_and_swap<uint32_t>(buffer));
      return X_E_SUCCESS;
    }
    // Causes dashboard to correctly process language/region change. It does not
    // contain any buffer.
    case 0x8000000D: {
      const bool is_pc_enabled =
          (kernel_state_->xconfig()->ReadSetting<uint8_t>(
               XCONFIG_USER_CATEGORY, XCONFIG_USER_PC_FLAGS) &
           X_PC_FLAGS::PCEnabled) != 0;

      return is_pc_enabled ? X_E_ACCESS_DENIED : X_E_SUCCESS;
    }
  }
  XELOGE(
      "Unimplemented XAM message app={:08X}, msg={:08X}, arg1={:08X}, "
      "arg2={:08X}",
      app_id(), message, buffer_ptr, buffer_length);
  return X_E_FAIL;
}

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace xe
