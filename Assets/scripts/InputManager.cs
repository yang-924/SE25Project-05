using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour
{
    // Start is called before the first frame update

    public static InputManager instance;

    private PlayerInput Input;

    private InputAction WAction;
    private InputAction SAction;
    private InputAction AAction;
    private InputAction DAction;
    private InputAction IAction;
    private InputAction KAction;
    private InputAction JAction;
    private InputAction LAction;
    private InputAction NAction;
    private InputAction F12Action;

    public bool F12Input;
    public bool WInput;
    public bool SInput;
    public bool AInput;
    public bool DInput;
    public bool IInput;
    public bool KInput;
    public bool JInput;
    public bool LInput;
    public bool NInput;

    private void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }

        Input = GetComponent<PlayerInput>();
        WAction = Input.actions["W"];
        SAction = Input.actions["S"];
        AAction = Input.actions["A"];
        DAction = Input.actions["D"];
        IAction = Input.actions["I"];
        KAction = Input.actions["K"];
        JAction = Input.actions["J"];
        LAction = Input.actions["L"];
        NAction = Input.actions["N"];
        F12Action = Input.actions["F12"];
    }

    // Update is called once per frame
    void Update()
    {
        WInput = WAction.ReadValue<float>() > 0.3f;
        AInput = AAction.ReadValue<float>() > 0.3f;
        SInput = SAction.ReadValue<float>() > 0.3f;
        DInput = DAction.ReadValue<float>() > 0.3f;
        IInput = IAction.ReadValue<float>() > 0.3f;
        JInput = JAction.ReadValue<float>() > 0.3f;
        KInput = KAction.ReadValue<float>() > 0.3f;
        LInput = LAction.ReadValue<float>() > 0.3f;
        NInput = NAction.ReadValue<float>() > 0.3f;
        F12Input = F12Action.ReadValue<float>() > 0.3f;
    }
}
