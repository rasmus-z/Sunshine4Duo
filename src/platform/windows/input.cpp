/**
 * @file src/platform/windows/input.cpp
 * @brief Definitions for input handling on Windows.
 */
#define WINVER 0x0A00

// platform includes
#include <Windows.h>

// standard includes
#include <cmath>
#include <thread>

// local includes
#include "keylayout.h"
#include "misc.h"
#include "src/config.h"
#include "src/globals.h"
#include "src/logging.h"
#include "src/platform/common.h"

#ifdef __MINGW32__
WINUSERAPI HSYNTHETICPOINTERDEVICE WINAPI CreateSyntheticPointerDevice(POINTER_INPUT_TYPE pointerType, ULONG maxCount, POINTER_FEEDBACK_MODE mode);
WINUSERAPI BOOL WINAPI InjectSyntheticPointerInput(HSYNTHETICPOINTERDEVICE device, CONST POINTER_TYPE_INFO *pointerInfo, UINT32 count);
WINUSERAPI VOID WINAPI DestroySyntheticPointerDevice(HSYNTHETICPOINTERDEVICE device);

/// <summary>
/// Defines the populated fields in a force feedback report.
/// </summary>
typedef enum _SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAGS
{
  SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_RIGHT_MOTOR_VALID = 0x1,
  SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_LEFT_MOTOR_VALID = 0x2,
  SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_RIGHT_TRIGGER_VALID = 0x4,
  SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_LEFT_TRIGGER_VALID = 0x8,
} SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAGS;

#pragma pack(push, 1)

/**
 * @brief The controller input report structure.
 */
typedef struct _DUO_CONTROLLER_INPUT_REPORT
{
  UINT8 Sync : 1; // Unused
  UINT8 Guide : 1;
  UINT8 Start : 1;
  UINT8 Back : 1;

  UINT8 A : 1;
  UINT8 B : 1;
  UINT8 X : 1;
  UINT8 Y : 1;

  UINT8 DPadUp : 1;
  UINT8 DPadDown : 1;
  UINT8 DPadLeft : 1;
  UINT8 DPadRight : 1;

  UINT8 LeftBumper : 1;
  UINT8 RightBumper : 1;
  UINT8 LeftStick : 1;
  UINT8 RightStick : 1;

  UINT16 LeftTrigger; // Analog 0-65535
  UINT16 RightTrigger; // Analog 0-65535

  INT16 LeftStickHorizontal; // -32768 (left) to 32767 (right)
  INT16 LeftStickVertical;   // -32768 (down) to 32767 (up)
  INT16 RightStickHorizontal; // -32768 (left) to 32767 (right)
  INT16 RightStickVertical;   // -32768 (down) to 32767 (up)
} DUO_CONTROLLER_INPUT_REPORT;

/**
 * @brief The controller force feedback report structure.
 */
typedef struct _DUO_CONTROLLER_FORCE_FEEDBACK_REPORT
{
  UINT8 Flags; // SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_RIGHT_MOTOR_VALID | SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_LEFT_MOTOR_VALID | SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_RIGHT_TRIGGER_VALID | SYNTHETIC_CONTROLLER_OUTPUT_REPORT_FLAG_LEFT_TRIGGER_VALID
  UINT8 LeftTrigger; // 0-255
  UINT8 RightTrigger; // 0-255
  UINT8 LeftMotor; // 0-255
  UINT8 RightMotor; // 0-255
  UINT8 Duration; // 0-255 (246 on test capture, must be 255 when combined with Delay)
  UINT8 Delay; // 0-255 (9 on test capture, must be 255 when combined with Duration)
  UINT8 Repeat; // 0 or 1 (0 on test capture)
  UINT8 PulsePeriod; // 0 on test capture
  UINT8 NumberOfPulses; // 235 on test capture
} DUO_CONTROLLER_FORCE_FEEDBACK_REPORT;

#pragma pack(pop)

/**
 * @brief Receives vibration data from a Duo controller.
 * @param controller The controller.
 * @param report The force feedback report.
 * @param context The context.
 */
typedef void (*DuoController_VibrationReportCallback_t)(void* controller, DUO_CONTROLLER_FORCE_FEEDBACK_REPORT* report, void* context);

/**
 * @brief Initializes the DuoController library.
 * @returns S_OK if the initialization was successful.
 */
WINUSERAPI HRESULT WINAPI DuoController_Initialize();

/**
 * @brief Uninitializes the DuoController library.
 * @returns S_OK if the uninitialization was successful.
 */
WINUSERAPI HRESULT WINAPI DuoController_Uninitialize();

/**
 * @brief Creates a new Duo controller.
 * @param vibrationCallback The vibration report callback.
 * @param vibrationCallbackContext The vibration callback context.
 * @param controller Receives the created controller.
 * @returns S_OK if the controller was created successfully.
 */
WINUSERAPI HRESULT WINAPI DuoController_CreateController(DuoController_VibrationReportCallback_t vibrationCallback, void* vibrationCallbackContext, void** controller);

/**
 * @brief Removes a Duo controller.
 * @param controller The controller to remove.
 * @returns S_OK if the controller was removed successfully.
 */
WINUSERAPI HRESULT WINAPI DuoController_RemoveController(void* controller);

/**
 * @brief Sends an input report to the given Duo controller.
 * @param controller The controller to send the input report to.
 * @param inputReport The input report to send.
 * @returns S_OK if the report was sent successfully.
 */
WINUSERAPI HRESULT WINAPI DuoController_SendReport(void* controller, DUO_CONTROLLER_INPUT_REPORT* inputReport);
#endif

namespace platf {
  using namespace std::literals;

  thread_local HDESK _lastKnownInputDesktop = nullptr;

  constexpr touch_port_t target_touch_port {
    0,
    0,
    65535,
    65535
  };

  struct gamepad_context_t {
    // The DuoController handle
    void* gp;

    // The feedback queue used to report back vibration
    feedback_queue_t feedback_queue;

    // The current DuoController state containing the buttons and axes
    DUO_CONTROLLER_INPUT_REPORT report;

    // The client relative index
    uint8_t client_relative_index;

    // The last reported rumble motor states
    gamepad_feedback_msg_t last_rumble;
  };

  class DuoController_t {
  public:
    /**
     * @brief Initializes the DuoController helper class.
     * @returns 0 if the initialization was successful.
     */
    int init()
    {
      // Load the DuoController.dll module
      mDuoController = LoadLibraryA("DuoController.dll");

      // We managed to load the DuoController.dll module
      if (mDuoController == NULL) {
        BOOST_LOG(fatal) << "DuoController library failed to load!"sv;
        return -1;
      }

      // Get pointers to the DuoController.dll functions
      fnDuoController_Initialize = (decltype(DuoController_Initialize) *) GetProcAddress(mDuoController, "DuoController_Initialize");
      fnDuoController_Uninitialize = (decltype(DuoController_Uninitialize) *) GetProcAddress(mDuoController, "DuoController_Uninitialize");
      fnDuoController_CreateController = (decltype(DuoController_CreateController) *) GetProcAddress(mDuoController, "DuoController_CreateController");
      fnDuoController_RemoveController = (decltype(DuoController_RemoveController) *) GetProcAddress(mDuoController, "DuoController_RemoveController");
      fnDuoController_SendReport = (decltype(DuoController_SendReport) *) GetProcAddress(mDuoController, "DuoController_SendReport");

      // Initialize the DuoController API
      if (fnDuoController_Initialize == NULL ||
          fnDuoController_Uninitialize == NULL ||
          fnDuoController_CreateController == NULL ||
          fnDuoController_RemoveController == NULL ||
          fnDuoController_SendReport == NULL) {
        // We failed to load the DuoController library
        BOOST_LOG(fatal) << "DuoController library is unsupported!"sv;
        return -1;
      }

      // Probe DuoController during startup so we can show an error in the UI *before* a stream starts.
      auto status = fnDuoController_Initialize();
      if (FAILED(status)) {
        // We failed to initialize the DuoController library
        BOOST_LOG(fatal) << "DuoController library failed to initialize! " << util::hex(status).to_string_view();
        return -1;
      }

      // Initialize the gamepad array
      gamepads.resize(MAX_GAMEPADS);

      // Return success
      return 0;
    }

    /**
     * @brief Allocates a new virtual gamepad.
     * @param id The gamepad index.
     * @param feedback_queue The feedback queue that will receive rumble events.
     * @returns 0 if the gamepad was successfully allocated.
     */
    int alloc_gamepad_internal(const gamepad_id_t &id, feedback_queue_t &feedback_queue)
    {
      // Cast the gamepad structure
      auto &gamepad = gamepads[id.globalIndex];

      // Ensure the slot isn't already in use
      assert(!gamepad.gp);

      // Assign the client relative index
      gamepad.client_relative_index = id.clientRelativeIndex;

      // Create the virtual gamepad
      auto status = fnDuoController_CreateController ? fnDuoController_CreateController(&duo_vibration_cb, this, &gamepad.gp) : E_FAIL;
      if (FAILED(status)) {
        BOOST_LOG(error) << "Could not create controller: " << util::hex(status).to_string_view();
        return -1;
      }

      // Initialize the gamepad report
      memset(&gamepad.report, 0, sizeof(gamepad.report));

      // Keep track of the feedback queue so we can report future rumble events
      gamepad.feedback_queue = std::move(feedback_queue);

      // Gamepad allocated successfully
      return 0;
    }

    /**
     * @brief Frees the given virtual gamepad.
     * @param nr The gamepad index.
     */
    void free_target(int nr)
    {
      // Cast the gamepad structure
      auto &gamepad = gamepads[nr];

      // The gamepad has been initialized
      if (gamepad.gp) {
        // Remove the virtual gamepad
        auto status = fnDuoController_RemoveController ? fnDuoController_RemoveController(gamepad.gp) : E_FAIL;
        if (FAILED(status)) {
          BOOST_LOG(warning) << "Could not remove controller: " << util::hex(status).to_string_view();
        }

        // Reset the internal handle
        gamepad.gp = NULL;
      }
    }

    /**
     * @brief Sends an updated gamepad report to the kernel.
     * @param nr The gamepad index.
     * @param report The updated gamepad report.
     */
    void SendReport(int nr, const DUO_CONTROLLER_INPUT_REPORT &report)
    {
      // Cast the gamepad structure
      auto &gamepad = gamepads[nr];

      // The gamepad has been initialized
      if (gamepad.gp) {
        // Send the gamepad report
        auto status = fnDuoController_SendReport ? fnDuoController_SendReport(gamepad.gp, const_cast<DUO_CONTROLLER_INPUT_REPORT*>(&report)) : E_FAIL;
        if (FAILED(status)) {
          BOOST_LOG(error) << "Could not send controller report: " << util::hex(status).to_string_view();
        }
      }
    }

    /**
     * @brief Destroys the DuoController helper class.
     */
    ~DuoController_t() {
      // Iterate all gamepads
      for (auto &gp : gamepads) {
        // Skip gamepads that aren't in use
        if (gp.gp) {
          // We have access to the DuoController module exports
          if (fnDuoController_RemoveController != NULL) {
            // Remove the controller
            fnDuoController_RemoveController(gp.gp);
          }

          // Reset the controller handle
          gp.gp = NULL;
        }
      }

      // We have access to the DuoController module exports
      if (fnDuoController_Uninitialize != NULL) {
        // Uninitialize the DuoController module
        fnDuoController_Uninitialize();
      }

      // We mapped the DuoController module
      if (mDuoController != NULL) {
        // Unmap the DuoController module
        FreeLibrary(mDuoController);

        // Reset the module handle
        mDuoController = NULL;

        // Reset the function pointers
        fnDuoController_Initialize = NULL;
        fnDuoController_Uninitialize = NULL;
        fnDuoController_CreateController = NULL;
        fnDuoController_RemoveController = NULL;
        fnDuoController_SendReport = NULL;
      }
    }

    // The virtual gamepad vector
    std::vector<gamepad_context_t> gamepads;
  private:
    // The DuoController module handle
    HMODULE mDuoController;

    // The DuoController module exports
    decltype(DuoController_Initialize) *fnDuoController_Initialize;
    decltype(DuoController_Uninitialize) *fnDuoController_Uninitialize;
    decltype(DuoController_CreateController) *fnDuoController_CreateController;
    decltype(DuoController_RemoveController) *fnDuoController_RemoveController;
    decltype(DuoController_SendReport) *fnDuoController_SendReport;

    /**
     * @brief Receives vibration data from the Windows kernel.
     * @param controller The internal controller handle.
     * @param smallMotorSpeed The small motor speed.
     * @param largeMotorSpeed The large motor speed.
     * @param context The callback context.
     */
    static void CALLBACK duo_vibration_cb(void *controller, DUO_CONTROLLER_FORCE_FEEDBACK_REPORT* report, void *context)
    {
      // Cast the DuoController instance
      auto *self = reinterpret_cast<DuoController_t*>(context);

      // Scale the motor values from 0~255 to 0~65535
      uint16_t low = static_cast<uint16_t>(report->RightMotor) << 8;
      uint16_t high = static_cast<uint16_t>(report->LeftMotor) << 8;

      // Iterate all allocated virtual gamepads
      for (int i = 0; i < self->gamepads.size(); i++)
      {
        // Cast the virtual gamepad
        auto &gp = self->gamepads[i];

        // We found the target virtual gamepad
        if (gp.gp == controller)
        {
          // Don't waste bandwidth reporting the same event over and over
          if (low != gp.last_rumble.data.rumble.highfreq || high != gp.last_rumble.data.rumble.lowfreq)
          {
            // Queue a rumble feedback message
            gamepad_feedback_msg_t msg = gamepad_feedback_msg_t::make_rumble(gp.client_relative_index, high, low);
            gp.feedback_queue->raise(msg);
            gp.last_rumble = msg;
          }

          // No reason to iterate the other virtual gamepads
          break;
        }
      }
    }
  };

  struct input_raw_t {
    ~input_raw_t() {
      delete duo;
    }

    DuoController_t *duo;

    decltype(CreateSyntheticPointerDevice) *fnCreateSyntheticPointerDevice;
    decltype(InjectSyntheticPointerInput) *fnInjectSyntheticPointerInput;
    decltype(DestroySyntheticPointerDevice) *fnDestroySyntheticPointerDevice;
  };

  input_t input() {
    input_t result {new input_raw_t {}};
    auto &raw = *(input_raw_t *) result.get();

    raw.duo = new DuoController_t {};
    if (raw.duo->init()) {
      delete raw.duo;
      raw.duo = nullptr;
    }

    // Get pointers to virtual touch/pen input functions (Win10 1809+)
    raw.fnCreateSyntheticPointerDevice = (decltype(CreateSyntheticPointerDevice) *) GetProcAddress(GetModuleHandleA("user32.dll"), "CreateSyntheticPointerDevice");
    raw.fnInjectSyntheticPointerInput = (decltype(InjectSyntheticPointerInput) *) GetProcAddress(GetModuleHandleA("user32.dll"), "InjectSyntheticPointerInput");
    raw.fnDestroySyntheticPointerDevice = (decltype(DestroySyntheticPointerDevice) *) GetProcAddress(GetModuleHandleA("user32.dll"), "DestroySyntheticPointerDevice");

    return result;
  }

  /**
   * @brief Calls SendInput() and switches input desktops if required.
   * @param i The `INPUT` struct to send.
   */
  void send_input(INPUT &i) {
  retry:
    auto send = SendInput(1, &i, sizeof(INPUT));
    if (send != 1) {
      auto hDesk = syncThreadDesktop();
      if (_lastKnownInputDesktop != hDesk) {
        _lastKnownInputDesktop = hDesk;
        goto retry;
      }
      BOOST_LOG(error) << "Couldn't send input"sv;
    }
  }

  /**
   * @brief Calls InjectSyntheticPointerInput() and switches input desktops if required.
   * @details Must only be called if InjectSyntheticPointerInput() is available.
   * @param input The global input context.
   * @param device The synthetic pointer device handle.
   * @param pointerInfo An array of `POINTER_TYPE_INFO` structs.
   * @param count The number of elements in `pointerInfo`.
   * @return true if input was successfully injected.
   */
  bool inject_synthetic_pointer_input(input_raw_t *input, HSYNTHETICPOINTERDEVICE device, const POINTER_TYPE_INFO *pointerInfo, UINT32 count) {
  retry:
    if (!input->fnInjectSyntheticPointerInput(device, pointerInfo, count)) {
      auto hDesk = syncThreadDesktop();
      if (_lastKnownInputDesktop != hDesk) {
        _lastKnownInputDesktop = hDesk;
        goto retry;
      }
      return false;
    }
    return true;
  }

  void abs_mouse(input_t &input, const touch_port_t &touch_port, float x, float y) {
    INPUT i {};

    i.type = INPUT_MOUSE;
    auto &mi = i.mi;

    mi.dwFlags =
      MOUSEEVENTF_MOVE |
      MOUSEEVENTF_ABSOLUTE |

      // MOUSEEVENTF_VIRTUALDESK maps to the entirety of the desktop rather than the primary desktop
      MOUSEEVENTF_VIRTUALDESK;

    auto scaled_x = std::lround((x + touch_port.offset_x) * ((float) target_touch_port.width / (float) touch_port.width));
    auto scaled_y = std::lround((y + touch_port.offset_y) * ((float) target_touch_port.height / (float) touch_port.height));

    mi.dx = scaled_x;
    mi.dy = scaled_y;

    send_input(i);
  }

  void move_mouse(input_t &input, int deltaX, int deltaY) {
    INPUT i {};

    i.type = INPUT_MOUSE;
    auto &mi = i.mi;

    mi.dwFlags = MOUSEEVENTF_MOVE;
    mi.dx = deltaX;
    mi.dy = deltaY;

    send_input(i);
  }

  util::point_t get_mouse_loc(input_t &input) {
    throw std::runtime_error("not implemented yet, has to pass tests");
    // TODO: Tests are failing, something wrong here?
    POINT p;
    if (!GetCursorPos(&p)) {
      return util::point_t {0.0, 0.0};
    }

    return util::point_t {
      (double) p.x,
      (double) p.y
    };
  }

  void button_mouse(input_t &input, int button, bool release) {
    INPUT i {};

    i.type = INPUT_MOUSE;
    auto &mi = i.mi;

    if (button == 1) {
      mi.dwFlags = release ? MOUSEEVENTF_LEFTUP : MOUSEEVENTF_LEFTDOWN;
    } else if (button == 2) {
      mi.dwFlags = release ? MOUSEEVENTF_MIDDLEUP : MOUSEEVENTF_MIDDLEDOWN;
    } else if (button == 3) {
      mi.dwFlags = release ? MOUSEEVENTF_RIGHTUP : MOUSEEVENTF_RIGHTDOWN;
    } else if (button == 4) {
      mi.dwFlags = release ? MOUSEEVENTF_XUP : MOUSEEVENTF_XDOWN;
      mi.mouseData = XBUTTON1;
    } else {
      mi.dwFlags = release ? MOUSEEVENTF_XUP : MOUSEEVENTF_XDOWN;
      mi.mouseData = XBUTTON2;
    }

    send_input(i);
  }

  void scroll(input_t &input, int distance) {
    INPUT i {};

    i.type = INPUT_MOUSE;
    auto &mi = i.mi;

    mi.dwFlags = MOUSEEVENTF_WHEEL;
    mi.mouseData = distance;

    send_input(i);
  }

  void hscroll(input_t &input, int distance) {
    INPUT i {};

    i.type = INPUT_MOUSE;
    auto &mi = i.mi;

    mi.dwFlags = MOUSEEVENTF_HWHEEL;
    mi.mouseData = distance;

    send_input(i);
  }

  void keyboard_update(input_t &input, uint16_t modcode, bool release, uint8_t flags) {
    INPUT i {};
    i.type = INPUT_KEYBOARD;
    auto &ki = i.ki;

    // If the client did not normalize this VK code to a US English layout, we can't accurately convert it to a scancode.
    // If we're set to always send scancodes, we will use the current keyboard layout to convert to a scancode. This will
    // assume the client and host have the same keyboard layout, but it's probably better than always using US English.
    if (!(flags & SS_KBE_FLAG_NON_NORMALIZED)) {
      // Mask off the extended key byte
      ki.wScan = VK_TO_SCANCODE_MAP[modcode & 0xFF];
    } else if (config::input.always_send_scancodes && modcode != VK_LWIN && modcode != VK_RWIN && modcode != VK_PAUSE) {
      // For some reason, MapVirtualKey(VK_LWIN, MAPVK_VK_TO_VSC) doesn't seem to work :/
      ki.wScan = MapVirtualKey(modcode, MAPVK_VK_TO_VSC);
    }

    // If we can map this to a scancode, send it as a scancode for maximum game compatibility.
    if (ki.wScan) {
      ki.dwFlags = KEYEVENTF_SCANCODE;
    } else {
      // If there is no scancode mapping or it's non-normalized, send it as a regular VK event.
      ki.wVk = modcode;
    }

    // https://docs.microsoft.com/en-us/windows/win32/inputdev/about-keyboard-input#keystroke-message-flags
    switch (modcode) {
      case VK_LWIN:
      case VK_RWIN:
      case VK_RMENU:
      case VK_RCONTROL:
      case VK_INSERT:
      case VK_DELETE:
      case VK_HOME:
      case VK_END:
      case VK_PRIOR:
      case VK_NEXT:
      case VK_UP:
      case VK_DOWN:
      case VK_LEFT:
      case VK_RIGHT:
      case VK_DIVIDE:
      case VK_APPS:
        ki.dwFlags |= KEYEVENTF_EXTENDEDKEY;
        break;
      default:
        break;
    }

    if (release) {
      ki.dwFlags |= KEYEVENTF_KEYUP;
    }

    send_input(i);
  }

  struct client_input_raw_t: public client_input_t {
    client_input_raw_t(input_t &input) {
      global = (input_raw_t *) input.get();
    }

    ~client_input_raw_t() override {
      if (penRepeatTask) {
        task_pool.cancel(penRepeatTask);
      }
      if (touchRepeatTask) {
        task_pool.cancel(touchRepeatTask);
      }

      if (pen) {
        global->fnDestroySyntheticPointerDevice(pen);
      }
      if (touch) {
        global->fnDestroySyntheticPointerDevice(touch);
      }
    }

    input_raw_t *global;

    // Device state and handles for pen and touch input must be stored in the per-client
    // input context, because each connected client may be sending their own independent
    // pen/touch events. To maintain separation, we expose separate pen and touch devices
    // for each client.

    HSYNTHETICPOINTERDEVICE pen {};
    POINTER_TYPE_INFO penInfo {};
    thread_pool_util::ThreadPool::task_id_t penRepeatTask {};

    HSYNTHETICPOINTERDEVICE touch {};
    POINTER_TYPE_INFO touchInfo[10] {};
    UINT32 activeTouchSlots {};
    thread_pool_util::ThreadPool::task_id_t touchRepeatTask {};
  };

  /**
   * @brief Allocates a context to store per-client input data.
   * @param input The global input context.
   * @return A unique pointer to a per-client input data context.
   */
  std::unique_ptr<client_input_t> allocate_client_input_context(input_t &input) {
    return std::make_unique<client_input_raw_t>(input);
  }

  /**
   * @brief Compacts the touch slots into a contiguous block and updates the active count.
   * @details Since this swaps entries around, all slot pointers/references are invalid after compaction.
   * @param raw The client-specific input context.
   */
  void perform_touch_compaction(client_input_raw_t *raw) {
    // Windows requires all active touches be contiguous when fed into InjectSyntheticPointerInput().
    UINT32 i;
    for (i = 0; i < ARRAYSIZE(raw->touchInfo); i++) {
      if (raw->touchInfo[i].touchInfo.pointerInfo.pointerFlags == POINTER_FLAG_NONE) {
        // This is an empty slot. Look for a later entry to move into this slot.
        for (UINT32 j = i + 1; j < ARRAYSIZE(raw->touchInfo); j++) {
          if (raw->touchInfo[j].touchInfo.pointerInfo.pointerFlags != POINTER_FLAG_NONE) {
            std::swap(raw->touchInfo[i], raw->touchInfo[j]);
            break;
          }
        }

        // If we didn't find anything, we've reached the end of active slots.
        if (raw->touchInfo[i].touchInfo.pointerInfo.pointerFlags == POINTER_FLAG_NONE) {
          break;
        }
      }
    }

    // Update the number of active touch slots
    raw->activeTouchSlots = i;
  }

  /**
   * @brief Gets a pointer slot by client-relative pointer ID, claiming a new one if necessary.
   * @param raw The raw client-specific input context.
   * @param pointerId The client's pointer ID.
   * @param eventType The LI_TOUCH_EVENT value from the client.
   * @return A pointer to the slot entry.
   */
  POINTER_TYPE_INFO *pointer_by_id(client_input_raw_t *raw, uint32_t pointerId, uint8_t eventType) {
    // Compact active touches into a single contiguous block
    perform_touch_compaction(raw);

    // Try to find a matching pointer ID
    for (UINT32 i = 0; i < ARRAYSIZE(raw->touchInfo); i++) {
      if (raw->touchInfo[i].touchInfo.pointerInfo.pointerId == pointerId &&
          raw->touchInfo[i].touchInfo.pointerInfo.pointerFlags != POINTER_FLAG_NONE) {
        if (eventType == LI_TOUCH_EVENT_DOWN && (raw->touchInfo[i].touchInfo.pointerInfo.pointerFlags & POINTER_FLAG_INCONTACT)) {
          BOOST_LOG(warning) << "Pointer "sv << pointerId << " already down. Did the client drop an up/cancel event?"sv;
        }

        return &raw->touchInfo[i];
      }
    }

    if (eventType != LI_TOUCH_EVENT_HOVER && eventType != LI_TOUCH_EVENT_DOWN) {
      BOOST_LOG(warning) << "Unexpected new pointer "sv << pointerId << " for event "sv << (uint32_t) eventType << ". Did the client drop a down/hover event?"sv;
    }

    // If there was none, grab an unused entry and increment the active slot count
    for (UINT32 i = 0; i < ARRAYSIZE(raw->touchInfo); i++) {
      if (raw->touchInfo[i].touchInfo.pointerInfo.pointerFlags == POINTER_FLAG_NONE) {
        raw->touchInfo[i].touchInfo.pointerInfo.pointerId = pointerId;
        raw->activeTouchSlots = i + 1;
        return &raw->touchInfo[i];
      }
    }

    return nullptr;
  }

  /**
   * @brief Populate common `POINTER_INFO` members shared between pen and touch events.
   * @param pointerInfo The pointer info to populate.
   * @param touchPort The current viewport for translating to screen coordinates.
   * @param eventType The type of touch/pen event.
   * @param x The normalized 0.0-1.0 X coordinate.
   * @param y The normalized 0.0-1.0 Y coordinate.
   */
  void populate_common_pointer_info(POINTER_INFO &pointerInfo, const touch_port_t &touchPort, uint8_t eventType, float x, float y) {
    switch (eventType) {
      case LI_TOUCH_EVENT_HOVER:
        pointerInfo.pointerFlags &= ~POINTER_FLAG_INCONTACT;
        pointerInfo.pointerFlags |= POINTER_FLAG_INRANGE | POINTER_FLAG_UPDATE;
        pointerInfo.ptPixelLocation.x = x * touchPort.width + touchPort.offset_x;
        pointerInfo.ptPixelLocation.y = y * touchPort.height + touchPort.offset_y;
        break;
      case LI_TOUCH_EVENT_DOWN:
        pointerInfo.pointerFlags |= POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT | POINTER_FLAG_DOWN;
        pointerInfo.ptPixelLocation.x = x * touchPort.width + touchPort.offset_x;
        pointerInfo.ptPixelLocation.y = y * touchPort.height + touchPort.offset_y;
        break;
      case LI_TOUCH_EVENT_UP:
        // We expect to get another LI_TOUCH_EVENT_HOVER if the pointer remains in range
        pointerInfo.pointerFlags &= ~(POINTER_FLAG_INCONTACT | POINTER_FLAG_INRANGE);
        pointerInfo.pointerFlags |= POINTER_FLAG_UP;
        break;
      case LI_TOUCH_EVENT_MOVE:
        pointerInfo.pointerFlags |= POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT | POINTER_FLAG_UPDATE;
        pointerInfo.ptPixelLocation.x = x * touchPort.width + touchPort.offset_x;
        pointerInfo.ptPixelLocation.y = y * touchPort.height + touchPort.offset_y;
        break;
      case LI_TOUCH_EVENT_CANCEL:
      case LI_TOUCH_EVENT_CANCEL_ALL:
        // If we were in contact with the touch surface at the time of the cancellation,
        // we'll set POINTER_FLAG_UP, otherwise set POINTER_FLAG_UPDATE.
        if (pointerInfo.pointerFlags & POINTER_FLAG_INCONTACT) {
          pointerInfo.pointerFlags |= POINTER_FLAG_UP;
        } else {
          pointerInfo.pointerFlags |= POINTER_FLAG_UPDATE;
        }
        pointerInfo.pointerFlags &= ~(POINTER_FLAG_INCONTACT | POINTER_FLAG_INRANGE);
        pointerInfo.pointerFlags |= POINTER_FLAG_CANCELED;
        break;
      case LI_TOUCH_EVENT_HOVER_LEAVE:
        pointerInfo.pointerFlags &= ~(POINTER_FLAG_INCONTACT | POINTER_FLAG_INRANGE);
        pointerInfo.pointerFlags |= POINTER_FLAG_UPDATE;
        break;
      case LI_TOUCH_EVENT_BUTTON_ONLY:
        // On Windows, we can only pass buttons if we have an active pointer
        if (pointerInfo.pointerFlags != POINTER_FLAG_NONE) {
          pointerInfo.pointerFlags |= POINTER_FLAG_UPDATE;
        }
        break;
      default:
        BOOST_LOG(warning) << "Unknown touch event: "sv << (uint32_t) eventType;
        break;
    }
  }

  // Active pointer interactions sent via InjectSyntheticPointerInput() seem to be automatically
  // cancelled by Windows if not repeated/updated within about a second. To avoid this, refresh
  // the injected input periodically.
  constexpr auto ISPI_REPEAT_INTERVAL = 50ms;

  /**
   * @brief Repeats the current touch state to avoid the interactions timing out.
   * @param raw The raw client-specific input context.
   */
  void repeat_touch(client_input_raw_t *raw) {
    if (!inject_synthetic_pointer_input(raw->global, raw->touch, raw->touchInfo, raw->activeTouchSlots)) {
      auto err = GetLastError();
      BOOST_LOG(warning) << "Failed to refresh virtual touch input: "sv << err;
    }

    raw->touchRepeatTask = task_pool.pushDelayed(repeat_touch, ISPI_REPEAT_INTERVAL, raw).task_id;
  }

  /**
   * @brief Repeats the current pen state to avoid the interactions timing out.
   * @param raw The raw client-specific input context.
   */
  void repeat_pen(client_input_raw_t *raw) {
    if (!inject_synthetic_pointer_input(raw->global, raw->pen, &raw->penInfo, 1)) {
      auto err = GetLastError();
      BOOST_LOG(warning) << "Failed to refresh virtual pen input: "sv << err;
    }

    raw->penRepeatTask = task_pool.pushDelayed(repeat_pen, ISPI_REPEAT_INTERVAL, raw).task_id;
  }

  /**
   * @brief Cancels all active touches.
   * @param raw The raw client-specific input context.
   */
  void cancel_all_active_touches(client_input_raw_t *raw) {
    // Cancel touch repeat callbacks
    if (raw->touchRepeatTask) {
      task_pool.cancel(raw->touchRepeatTask);
      raw->touchRepeatTask = nullptr;
    }

    // Compact touches to update activeTouchSlots
    perform_touch_compaction(raw);

    // If we have active slots, cancel them all
    if (raw->activeTouchSlots > 0) {
      for (UINT32 i = 0; i < raw->activeTouchSlots; i++) {
        populate_common_pointer_info(raw->touchInfo[i].touchInfo.pointerInfo, {}, LI_TOUCH_EVENT_CANCEL_ALL, 0.0f, 0.0f);
        raw->touchInfo[i].touchInfo.touchMask = TOUCH_MASK_NONE;
      }
      if (!inject_synthetic_pointer_input(raw->global, raw->touch, raw->touchInfo, raw->activeTouchSlots)) {
        auto err = GetLastError();
        BOOST_LOG(warning) << "Failed to cancel all virtual touch input: "sv << err;
      }
    }

    // Zero all touch state
    std::memset(raw->touchInfo, 0, sizeof(raw->touchInfo));
    raw->activeTouchSlots = 0;
  }

  // These are edge-triggered pointer state flags that should always be cleared next frame
  constexpr auto EDGE_TRIGGERED_POINTER_FLAGS = POINTER_FLAG_DOWN | POINTER_FLAG_UP | POINTER_FLAG_CANCELED | POINTER_FLAG_UPDATE;

  /**
   * @brief Sends a touch event to the OS.
   * @param input The client-specific input context.
   * @param touch_port The current viewport for translating to screen coordinates.
   * @param touch The touch event.
   */
  void touch_update(client_input_t *input, const touch_port_t &touch_port, const touch_input_t &touch) {
    auto raw = (client_input_raw_t *) input;

    // Bail if we're not running on an OS that supports virtual touch input
    if (!raw->global->fnCreateSyntheticPointerDevice ||
        !raw->global->fnInjectSyntheticPointerInput ||
        !raw->global->fnDestroySyntheticPointerDevice) {
      BOOST_LOG(warning) << "Touch input requires Windows 10 1809 or later"sv;
      return;
    }

    // If there's not already a virtual touch device, create one now
    if (!raw->touch) {
      if (touch.eventType != LI_TOUCH_EVENT_CANCEL_ALL) {
        BOOST_LOG(info) << "Creating virtual touch input device"sv;
        raw->touch = raw->global->fnCreateSyntheticPointerDevice(PT_TOUCH, ARRAYSIZE(raw->touchInfo), POINTER_FEEDBACK_DEFAULT);
        if (!raw->touch) {
          auto err = GetLastError();
          BOOST_LOG(warning) << "Failed to create virtual touch device: "sv << err;
          return;
        }
      } else {
        // No need to cancel anything if we had no touch input device
        return;
      }
    }

    // Cancel touch repeat callbacks
    if (raw->touchRepeatTask) {
      task_pool.cancel(raw->touchRepeatTask);
      raw->touchRepeatTask = nullptr;
    }

    // If this is a special request to cancel all touches, do that and return
    if (touch.eventType == LI_TOUCH_EVENT_CANCEL_ALL) {
      cancel_all_active_touches(raw);
      return;
    }

    // Find or allocate an entry for this touch pointer ID
    auto pointer = pointer_by_id(raw, touch.pointerId, touch.eventType);
    if (!pointer) {
      BOOST_LOG(error) << "No unused pointer entries! Cancelling all active touches!"sv;
      cancel_all_active_touches(raw);
      pointer = pointer_by_id(raw, touch.pointerId, touch.eventType);
    }

    pointer->type = PT_TOUCH;

    auto &touchInfo = pointer->touchInfo;
    touchInfo.pointerInfo.pointerType = PT_TOUCH;

    // Populate shared pointer info fields
    populate_common_pointer_info(touchInfo.pointerInfo, touch_port, touch.eventType, touch.x, touch.y);

    touchInfo.touchMask = TOUCH_MASK_NONE;

    // Pressure and contact area only apply to in-contact pointers.
    //
    // The clients also pass distance and tool size for hovers, but Windows doesn't
    // provide APIs to receive that data.
    if (touchInfo.pointerInfo.pointerFlags & POINTER_FLAG_INCONTACT) {
      if (touch.pressureOrDistance != 0.0f) {
        touchInfo.touchMask |= TOUCH_MASK_PRESSURE;

        // Convert the 0.0f..1.0f float to the 0..1024 range that Windows uses
        touchInfo.pressure = (UINT32) (touch.pressureOrDistance * 1024);
      } else {
        // The default touch pressure is 512
        touchInfo.pressure = 512;
      }

      if (touch.contactAreaMajor != 0.0f && touch.contactAreaMinor != 0.0f) {
        // For the purposes of contact area calculation, we will assume the touches
        // are at a 45 degree angle if rotation is unknown. This will scale the major
        // axis value by width and height equally.
        float rotationAngleDegs = touch.rotation == LI_ROT_UNKNOWN ? 45 : touch.rotation;

        float majorAxisAngle = rotationAngleDegs * (M_PI / 180);
        float minorAxisAngle = majorAxisAngle + (M_PI / 2);

        // Estimate the contact rectangle
        float contactWidth = (std::cos(majorAxisAngle) * touch.contactAreaMajor) + (std::cos(minorAxisAngle) * touch.contactAreaMinor);
        float contactHeight = (std::sin(majorAxisAngle) * touch.contactAreaMajor) + (std::sin(minorAxisAngle) * touch.contactAreaMinor);

        // Convert into screen coordinates centered at the touch location and constrained by screen dimensions
        touchInfo.rcContact.left = std::max<LONG>(touch_port.offset_x, touchInfo.pointerInfo.ptPixelLocation.x - std::floor(contactWidth / 2));
        touchInfo.rcContact.right = std::min<LONG>(touch_port.offset_x + touch_port.width, touchInfo.pointerInfo.ptPixelLocation.x + std::ceil(contactWidth / 2));
        touchInfo.rcContact.top = std::max<LONG>(touch_port.offset_y, touchInfo.pointerInfo.ptPixelLocation.y - std::floor(contactHeight / 2));
        touchInfo.rcContact.bottom = std::min<LONG>(touch_port.offset_y + touch_port.height, touchInfo.pointerInfo.ptPixelLocation.y + std::ceil(contactHeight / 2));

        touchInfo.touchMask |= TOUCH_MASK_CONTACTAREA;
      }
    } else {
      touchInfo.pressure = 0;
      touchInfo.rcContact = {};
    }

    if (touch.rotation != LI_ROT_UNKNOWN) {
      touchInfo.touchMask |= TOUCH_MASK_ORIENTATION;
      touchInfo.orientation = touch.rotation;
    } else {
      touchInfo.orientation = 0;
    }

    if (!inject_synthetic_pointer_input(raw->global, raw->touch, raw->touchInfo, raw->activeTouchSlots)) {
      auto err = GetLastError();
      BOOST_LOG(warning) << "Failed to inject virtual touch input: "sv << err;
      return;
    }

    // Clear pointer flags that should only remain set for one frame
    touchInfo.pointerInfo.pointerFlags &= ~EDGE_TRIGGERED_POINTER_FLAGS;

    // If we still have an active touch, refresh the touch state periodically
    if (raw->activeTouchSlots > 1 || touchInfo.pointerInfo.pointerFlags != POINTER_FLAG_NONE) {
      raw->touchRepeatTask = task_pool.pushDelayed(repeat_touch, ISPI_REPEAT_INTERVAL, raw).task_id;
    }
  }

  /**
   * @brief Sends a pen event to the OS.
   * @param input The client-specific input context.
   * @param touch_port The current viewport for translating to screen coordinates.
   * @param pen The pen event.
   */
  void pen_update(client_input_t *input, const touch_port_t &touch_port, const pen_input_t &pen) {
    auto raw = (client_input_raw_t *) input;

    // Bail if we're not running on an OS that supports virtual pen input
    if (!raw->global->fnCreateSyntheticPointerDevice ||
        !raw->global->fnInjectSyntheticPointerInput ||
        !raw->global->fnDestroySyntheticPointerDevice) {
      BOOST_LOG(warning) << "Pen input requires Windows 10 1809 or later"sv;
      return;
    }

    // If there's not already a virtual pen device, create one now
    if (!raw->pen) {
      if (pen.eventType != LI_TOUCH_EVENT_CANCEL_ALL) {
        BOOST_LOG(info) << "Creating virtual pen input device"sv;
        raw->pen = raw->global->fnCreateSyntheticPointerDevice(PT_PEN, 1, POINTER_FEEDBACK_DEFAULT);
        if (!raw->pen) {
          auto err = GetLastError();
          BOOST_LOG(warning) << "Failed to create virtual pen device: "sv << err;
          return;
        }
      } else {
        // No need to cancel anything if we had no pen input device
        return;
      }
    }

    // Cancel pen repeat callbacks
    if (raw->penRepeatTask) {
      task_pool.cancel(raw->penRepeatTask);
      raw->penRepeatTask = nullptr;
    }

    raw->penInfo.type = PT_PEN;

    auto &penInfo = raw->penInfo.penInfo;
    penInfo.pointerInfo.pointerType = PT_PEN;
    penInfo.pointerInfo.pointerId = 0;

    // Populate shared pointer info fields
    populate_common_pointer_info(penInfo.pointerInfo, touch_port, pen.eventType, pen.x, pen.y);

    // Windows only supports a single pen button, so send all buttons as the barrel button
    if (pen.penButtons) {
      penInfo.penFlags |= PEN_FLAG_BARREL;
    } else {
      penInfo.penFlags &= ~PEN_FLAG_BARREL;
    }

    switch (pen.toolType) {
      default:
      case LI_TOOL_TYPE_PEN:
        penInfo.penFlags &= ~PEN_FLAG_ERASER;
        break;
      case LI_TOOL_TYPE_ERASER:
        penInfo.penFlags |= PEN_FLAG_ERASER;
        break;
      case LI_TOOL_TYPE_UNKNOWN:
        // Leave tool flags alone
        break;
    }

    penInfo.penMask = PEN_MASK_NONE;

    // Windows doesn't support hover distance, so only pass pressure/distance when the pointer is in contact
    if ((penInfo.pointerInfo.pointerFlags & POINTER_FLAG_INCONTACT) && pen.pressureOrDistance != 0.0f) {
      penInfo.penMask |= PEN_MASK_PRESSURE;

      // Convert the 0.0f..1.0f float to the 0..1024 range that Windows uses
      penInfo.pressure = (UINT32) (pen.pressureOrDistance * 1024);
    } else {
      // The default pen pressure is 0
      penInfo.pressure = 0;
    }

    if (pen.rotation != LI_ROT_UNKNOWN) {
      penInfo.penMask |= PEN_MASK_ROTATION;
      penInfo.rotation = pen.rotation;
    } else {
      penInfo.rotation = 0;
    }

    // We require rotation and tilt to perform the conversion to X and Y tilt angles
    if (pen.tilt != LI_TILT_UNKNOWN && pen.rotation != LI_ROT_UNKNOWN) {
      auto rotationRads = pen.rotation * (M_PI / 180.f);
      auto tiltRads = pen.tilt * (M_PI / 180.f);
      auto r = std::sin(tiltRads);
      auto z = std::cos(tiltRads);

      // Convert polar coordinates into X and Y tilt angles
      penInfo.penMask |= PEN_MASK_TILT_X | PEN_MASK_TILT_Y;
      penInfo.tiltX = (INT32) (std::atan2(std::sin(-rotationRads) * r, z) * 180.f / M_PI);
      penInfo.tiltY = (INT32) (std::atan2(std::cos(-rotationRads) * r, z) * 180.f / M_PI);
    } else {
      penInfo.tiltX = 0;
      penInfo.tiltY = 0;
    }

    if (!inject_synthetic_pointer_input(raw->global, raw->pen, &raw->penInfo, 1)) {
      auto err = GetLastError();
      BOOST_LOG(warning) << "Failed to inject virtual pen input: "sv << err;
      return;
    }

    // Clear pointer flags that should only remain set for one frame
    penInfo.pointerInfo.pointerFlags &= ~EDGE_TRIGGERED_POINTER_FLAGS;

    // If we still have an active pen interaction, refresh the pen state periodically
    if (penInfo.pointerInfo.pointerFlags != POINTER_FLAG_NONE) {
      raw->penRepeatTask = task_pool.pushDelayed(repeat_pen, ISPI_REPEAT_INTERVAL, raw).task_id;
    }
  }

  void unicode(input_t &input, char *utf8, int size) {
    // We can do no worse than one UTF-16 character per byte of UTF-8
    WCHAR wide[size];

    int chars = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8, size, wide, size);
    if (chars <= 0) {
      return;
    }

    // Send all key down events
    for (int i = 0; i < chars; i++) {
      INPUT input {};
      input.type = INPUT_KEYBOARD;
      input.ki.wScan = wide[i];
      input.ki.dwFlags = KEYEVENTF_UNICODE;
      send_input(input);
    }

    // Send all key up events
    for (int i = 0; i < chars; i++) {
      INPUT input {};
      input.type = INPUT_KEYBOARD;
      input.ki.wScan = wide[i];
      input.ki.dwFlags = KEYEVENTF_UNICODE | KEYEVENTF_KEYUP;
      send_input(input);
    }
  }

  /**
   * @brief Allocates a new virtual gamepad.
   * @param input The raw input structure.
   * @param id The virtual gamepad index.
   * @param metadata The virtual gamepad metadata.
   * @param feedback_queue The feedback queue used to transmit virtual gamepad output data.
   */
  int alloc_gamepad(input_t &input, const gamepad_id_t &id, const gamepad_arrival_t &metadata, feedback_queue_t feedback_queue) {
    // Cast the raw input structure
    auto raw = (input_raw_t *) input.get();

    // Allocate a new virtual gamepad
    return raw->duo->alloc_gamepad_internal(id, feedback_queue);
  }

  /**
   * @brief Frees the given virtual gamepad.
   * @param input The raw input structure.
   * @param nr The virtual gamepad index.
   */
  void free_gamepad(input_t &input, int nr) {
    // Cast the raw input structure
    auto raw = (input_raw_t *) input.get();

    // Free the virtual gamepad
    raw->duo->free_target(nr);
  }

  /**
   * @brief Snaps to the analog stick axis boundary.
   * @param val The analog stick axis.
   * @return The snapped analog stick axis.
   */
  static inline int16_t snap_to_analog_stick_axis_boundary(int16_t val)
  {
    // Distance to the lower bound (INT16_MIN = -32768)
    const int32_t dist_low  = (int32_t)val - INT16_MIN;

    // Distance to the upper bound (INT16_MAX = 32767)
    const int32_t dist_high = INT16_MAX - val;

    // We're close enough to the lower boundary
    if (dist_low <= 10) {
      // Snap to the lower boundary
      return INT16_MIN;
    }

    // We're close enough to the upper boundary
    if (dist_high <= 10) {
      // Snap to the upper boundary
      return INT16_MAX;
    }

    // Keep the original value
    return val;
  }

  /**
   * @brief Updates the DuoController input report with the provided gamepad state.
   * @param gamepad The gamepad to update.
   * @param gamepad_state The gamepad button/axis state sent from the client.
   */
  static void duo_update_state(gamepad_context_t &gamepad, const gamepad_state_t &gamepad_state) {
    auto &report = gamepad.report;

    report.DPadUp = (gamepad_state.buttonFlags & DPAD_UP) != 0;
    report.DPadDown = (gamepad_state.buttonFlags & DPAD_DOWN) != 0;
    report.DPadLeft = (gamepad_state.buttonFlags & DPAD_LEFT) != 0;
    report.DPadRight = (gamepad_state.buttonFlags & DPAD_RIGHT) != 0;
    report.Start = (gamepad_state.buttonFlags & START) != 0;
    report.Back = (gamepad_state.buttonFlags & BACK) != 0;
    report.LeftStick = (gamepad_state.buttonFlags & LEFT_STICK) != 0;
    report.RightStick = (gamepad_state.buttonFlags & RIGHT_STICK) != 0;
    report.LeftBumper = (gamepad_state.buttonFlags & LEFT_BUTTON) != 0;
    report.RightBumper = (gamepad_state.buttonFlags & RIGHT_BUTTON) != 0;
    report.Guide = (gamepad_state.buttonFlags & (HOME | MISC_BUTTON)) != 0;
    report.A = (gamepad_state.buttonFlags & A) != 0;
    report.B = (gamepad_state.buttonFlags & B) != 0;
    report.X = (gamepad_state.buttonFlags & X) != 0;
    report.Y = (gamepad_state.buttonFlags & Y) != 0;
    report.LeftTrigger = static_cast<UINT16>(static_cast<double>(gamepad_state.lt) / 255.0f * 65535.0f);
    report.RightTrigger = static_cast<UINT16>(static_cast<double>(gamepad_state.rt) / 255.0f * 65535.0f);
    report.LeftStickHorizontal = snap_to_analog_stick_axis_boundary(gamepad_state.lsX);
    report.LeftStickVertical = snap_to_analog_stick_axis_boundary(gamepad_state.lsY);
    report.RightStickHorizontal = snap_to_analog_stick_axis_boundary(gamepad_state.rsX);
    report.RightStickVertical = snap_to_analog_stick_axis_boundary(gamepad_state.rsY);
  }

  /**
   * @brief Updates virtual gamepad with the provided gamepad state.
   * @param input The input context.
   * @param nr The gamepad index to update.
   * @param gamepad_state The gamepad button/axis state sent from the client.
   */
  void gamepad_update(input_t &input, int nr, const gamepad_state_t &gamepad_state) {
    auto duo = ((input_raw_t *) input.get())->duo;

    auto &gamepad = duo->gamepads[nr];
    duo_update_state(gamepad, gamepad_state);
    duo->SendReport(nr, gamepad.report);
  }

  /**
   * @brief Sends a gamepad touch event to the OS.
   * @param input The global input context.
   * @param touch The touch event.
   */
  void gamepad_touch(input_t &input, const gamepad_touch_t &touch) {
    // There's no good way to map this to synthetic gamepads (yet)
    return;
  }

  /**
   * @brief Sends a gamepad motion event to the OS.
   * @param input The global input context.
   * @param motion The motion event.
   */
  void gamepad_motion(input_t &input, const gamepad_motion_t &motion) {
    // There's no good way to map this to synthetic gamepads (yet)
    return;
  }

  /**
   * @brief Sends a gamepad battery event to the OS.
   * @param input The global input context.
   * @param battery The battery event.
   */
  void gamepad_battery(input_t &input, const gamepad_battery_t &battery) {
    // Synthetic gamepads have no battery
    return;
  }

  void freeInput(void *p) {
    auto input = (input_raw_t *) p;

    delete input;
  }

  std::vector<supported_gamepad_t> &supported_gamepads(input_t *input) {
    static std::vector gps {
      supported_gamepad_t {"auto", true, ""},
      supported_gamepad_t {"x360", true, ""},
      supported_gamepad_t {"ds4", false, "gamepads.type-not-supported"},
    };

    for (auto &[name, is_enabled, reason_disabled] : gps) {
      if (!is_enabled) {
        BOOST_LOG(warning) << "Gamepad " << name << " is disabled due to " << reason_disabled;
      }
    }

    return gps;
  }

  /**
   * @brief Returns the supported platform capabilities to advertise to the client.
   * @return Capability flags.
   */
  platform_caps::caps_t get_capabilities() {
    platform_caps::caps_t caps = 0;

    // We support controller touchpad input as long as we're not emulating X360
    if (config::input.gamepad != "x360"sv) {
      caps |= platform_caps::controller_touch;
    }

    // We support pen and touch input on Win10 1809+
    if (GetProcAddress(GetModuleHandleA("user32.dll"), "CreateSyntheticPointerDevice") != nullptr) {
      if (config::input.native_pen_touch) {
        caps |= platform_caps::pen_touch;
      }
    } else {
      BOOST_LOG(warning) << "Touch input requires Windows 10 1809 or later"sv;
    }

    return caps;
  }
}  // namespace platf
