////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

const int KEYS_TO_AWE_KEYS[] = {
    0x1B, // GLFW_KEY_ESCAPE
    0x0D, // GLFW_KEY_ENTER
    0x09, // GLFW_KEY_TAB
    0x08, // GLFW_KEY_BACKSPACE
    0x2D, // GLFW_KEY_INSERT
    0x2E, // GLFE_KEY_DELETE
    0x27, // GLFW_KEY_RIGHT
    0x25, // GLFW_KEY_LEFT
    0x28, // GLFW_KEY_DOWN
    0x26, // GLFW_KEY_UP
    0x21, // GLFW_KEY_PAGE_UP
    0x22, // GLFW_KEY_PAGE_DOWN
    0x24, // GLFW_KEY_HOME
    0x23, // GLFW_KEY_END

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    0x14, // GLFW_KEY_CAPS_LOCK
    0x91, // GLFW_KEY_SCROLL_LOCK
    0x90, // GLFW_KEY_NUM_LOCK
    0x2A, // GLFW_KEY_PRINT_SCREEN
    0x13, // GLFW_KEY_PAUSE

    0x00, 0x00, 0x00, 0x00, 0x00,

    0x70, // GLFW_KEY_F1
    0x71, // GLFW_KEY_F2
    0x72, // GLFW_KEY_F3
    0x73, // GLFW_KEY_F4
    0x74, // GLFW_KEY_F5
    0x75, // GLFW_KEY_F6
    0x76, // GLFW_KEY_F7
    0x77, // GLFW_KEY_F8
    0x78, // GLFW_KEY_F9
    0x79, // GLFW_KEY_F10
    0x7A, // GLFW_KEY_F11
    0x7B, // GLFW_KEY_F12
    0x7C, // GLFW_KEY_F13
    0x7D, // GLFW_KEY_F14
    0x7E, // GLFW_KEY_F15
    0x7F, // GLFW_KEY_F16
    0x80, // GLFW_KEY_F17
    0x81, // GLFW_KEY_F18
    0x82, // GLFW_KEY_F19
    0x83, // GLFW_KEY_F20
    0x84, // GLFW_KEY_F21
    0x85, // GLFW_KEY_F22
    0x86, // GLFW_KEY_F23
    0x87, // GLFW_KEY_F24
    0x00, // GLFW_KEY_F25 -- No awe equivalent

    0x00, 0x00, 0x00, 0x00, 0x00,

    0x60, // GLFW_KEY_KP_0
    0x61, // GLFW_KEY_KP_1
    0x62, // GLFW_KEY_KP_2
    0x63, // GLFW_KEY_KP_3
    0x64, // GLFW_KEY_KP_4
    0x65, // GLFW_KEY_KP_5
    0x66, // GLFW_KEY_KP_6
    0x67, // GLFW_KEY_KP_7
    0x68, // GLFW_KEY_KP_8
    0x69, // GLFW_KEY_KP_9
    0x6E, // GLFW_KEY_KP_DECIMAL
    0x6F, // GLFW_KEY_KP_DIVIDE
    0x6A, // GLFW_KEY_KP_MULTIPLY
    0x6D, // GLFW_KEY_KP_SUBTRACT
    0x6B, // GLFW_KEY_KP_ADD
    0x6C, // GLFW_KEY_KP_ENTER
    0x00, // GLFW_KEY_KP_EQUAL -- No awe equivalent

    0x00, 0x00, 0x00,

    0xA0, // GLFW_KEY_LEFT_SHIFT
    0xA2, // GLFW_KEY_LEFT_CONTROL
    0xA4, // GLFW_KEY_LEFT_ALT
    0x5B, // GLFW_KEY_LEFT_SUPER
    0xA1, // GLFW_KEY_RIGHT_SHIFT
    0xA3, // GLFW_KEY_RIGHT_CONTROL
    0xA5, // GLFW_KEY_RIGHT_ALT
    0x5C, // GLFW_KEY_RIGHT_SUPER
    0x12  // GLFW_KEY_MENU
};

int key_to_awe_key(Key key)
{
    switch(key)
    {
    case Key::SPACE:
        return 0x20;
    case Key::APOSTROPHE:
        return 0xBA;
    case Key::COMMA:
        return 0xBC;
    case Key::MINUS:
        return 0xBD;
    case Key::PERIOD:
        return 0xBE;
    case Key::SLASH:
        return 0xBF;
    case Key::KEY_0:
    case Key::KEY_1:
    case Key::KEY_2:
    case Key::KEY_3:
    case Key::KEY_4:
    case Key::KEY_5:
    case Key::KEY_6:
    case Key::KEY_7:
    case Key::KEY_8:
    case Key::KEY_9:
        return static_cast<int>(key);
    case Key::SEMICOLON:
        return 0x00;
    case Key::EQUAL:
        return 0xBB;
    case Key::A:
    case Key::B:
    case Key::C:
    case Key::D:
    case Key::E:
    case Key::F:
    case Key::G:
    case Key::H:
    case Key::I:
    case Key::J:
    case Key::K:
    case Key::L:
    case Key::M:
    case Key::N:
    case Key::O:
    case Key::P:
    case Key::Q:
    case Key::R:
    case Key::S:
    case Key::T:
    case Key::U:
    case Key::V:
    case Key::W:
    case Key::X:
    case Key::Y:
    case Key::Z:
        return static_cast<int>(key);
    case Key::LEFT_BRACKET:
        return 0x00;
    case Key::BACKSLASH:
        return 0x00;
    case Key::RIGHT_BRACKET:
        return 0x00;
    case Key::GRAVE_ACCENT:
        return 0x00;
    case Key::WORLD_1:
        return 0x00;
    case Key::WORLD_2:
        return 0x00;
    default:
        return KEYS_TO_AWE_KEYS[static_cast<int>(key) - static_cast<int>(Key::ESCAPE)];
    }
}

class AweKeyEvent : public Awesomium::WebKeyboardEvent
{
    ///////////////////////////////////////////////////////////////////////////////
    // ----------------------------------------------------------- public interface
  public:
    // ----------------------------------------------------- contruction interface
    AweKeyEvent(unsigned c)
    {
        type = Awesomium::WebKeyboardEvent::kTypeChar;
        modifiers = 0;
        virtual_key_code = static_cast<int>(c);
        native_key_code = 0;
        text[0] = static_cast<wchar16>(c);
    }

    AweKeyEvent(Key key, int scancode, int action, int mods)
    {
        int key_awe = key_to_awe_key(key);

        auto awe_type = Awesomium::WebKeyboardEvent::kTypeKeyUp;
        auto awe_mods = mods;

        if(action != 0)
        {
            awe_type = Awesomium::WebKeyboardEvent::kTypeKeyDown;
        }

        if(action == 2)
        {
            awe_mods |= Awesomium::WebKeyboardEvent::kModIsAutorepeat;
        }

        type = awe_type;
        modifiers = awe_mods;
        virtual_key_code = key_awe;
        native_key_code = static_cast<int>(key);
        is_system_key = false;

        char* buf = new char[20];
        Awesomium::GetKeyIdentifierFromVirtualKeyCode(key_awe, &buf);
        strcpy(key_identifier, buf);
        delete[] buf;
    }
};
