using UnityEngine;
using UnityEngine.InputSystem;

public class HybridController : MonoBehaviour
{
    [SerializeField]
    public InputActionReference throttleIA, aileronIA, elevatorIA, rudderIA, UMotorsIA, UForBackIA, ULeftRightIA;
    public GameObject go;
    public GameObject Camera;

    [ReadOnly]
    public float Ut;
    [ReadOnly]
    public float Ua;
    [ReadOnly]
    public float Ue;
    [ReadOnly]
    public float Ur;

    
    [ReadOnly]
    public float URotorBase;
    [ReadOnly]
    public float U1;
    [ReadOnly]
    public float U2;
    [ReadOnly]
    public float U3;
    [ReadOnly]
    public float U4;
    [ReadOnly]
    public Vector3 position;
    [ReadOnly]
    public Vector3 velocity;
    [ReadOnly]
    public Vector3 eulerAngles;
    [ReadOnly]
    public Quaternion rotation;
    [ReadOnly]
    public Vector3 angularVelocity;

    
    public float UeMinMax;
    public float UaMinMax;
    public float UrMinMax;
    public float UiVal;
    public float UiDeltaVal;
    public float UtVal;


    private void Awake() {
        throttleIA.action.performed += ThrottlePressed;
        throttleIA.action.canceled  += ThrottleReleased;

        UMotorsIA.action.performed += MotorsPressed;
        UMotorsIA.action.canceled  += MotorsReleased;
    }
    
    // Start is called before the first frame update
    private void Start()
    {
        // Setup initial condition
        Ut = 0.0f;
        Ua = 0.0f;
        Ue = 0.0f;
        Ur = 0.0f;
        URotorBase = UiVal;

        position = new Vector3(0, 0, 0);
        go.transform.position = position;

        velocity = new Vector3(0, 0, 0);

        eulerAngles = new Vector3(0, 0, 0);
        go.transform.eulerAngles = eulerAngles;

        angularVelocity = new Vector3(0, 0, 0);
    }

    private void FixedUpdate()
    {        
        // Get control from inputs
        float newUe = Ue + elevatorIA.action.ReadValue<float>();
        Ue = Mathf.Clamp(newUe, -UeMinMax, UeMinMax);
        
        float newUa = aileronIA.action.ReadValue<float>();
        Ua = Mathf.Clamp(newUa, -UaMinMax, UaMinMax);

        float newUr = Ur + rudderIA.action.ReadValue<float>();
        Ur = Mathf.Clamp(newUr, -UrMinMax, UrMinMax);
        
        float ForBack = UForBackIA.action.ReadValue<float>();
        float LeftRight = ULeftRightIA.action.ReadValue<float>();

        U1 = U2 = U3 = U4 = URotorBase;
        if (ForBack > 0){
            U3 = U4 += UiDeltaVal;
        } else if (ForBack < 0) {
            U1 = U2 += UiDeltaVal;
        }

        if (LeftRight > 0){
            U2 = U3 += UiDeltaVal;
        } else if (LeftRight < 0) {
            U1 = U4 += UiDeltaVal;
        }

        // Setup state variables from internal state
        float x =       position.x;
        float y =       position.y;
        float z =       position.z;
        float u =       velocity.x;
        float v =       velocity.y;
        float w =       velocity.z;
        float phi =     eulerAngles.x     * Mathf.Deg2Rad;
        float theta =   eulerAngles.y     * Mathf.Deg2Rad;
        float psi =     eulerAngles.z     * Mathf.Deg2Rad;
        float p =       angularVelocity.x * Mathf.Deg2Rad;
        float q =       angularVelocity.y * Mathf.Deg2Rad;
        float r =       angularVelocity.z * Mathf.Deg2Rad;
                
        HybridDynamicsMarta dynamics = new(x, y, z, u, v, w, phi, theta, psi, p, q, r, Ua, Ue, Ur, Ut, U1, U2, U3, U4);

        // Calculate derivatives
        // derivatives = {dx, dy, dz, du, dv, dw, dphi, dtheta, dpsi, dp, dq, dr}
        double[] derivatives = dynamics.GetDerivatives();
        float dx =      (float) derivatives[0]  * Time.fixedDeltaTime;
        float dy =      (float) derivatives[1]  * Time.fixedDeltaTime;
        float dz =      (float) derivatives[2]  * Time.fixedDeltaTime;
        float du =      (float) derivatives[3]  * Time.fixedDeltaTime;
        float dv =      (float) derivatives[4]  * Time.fixedDeltaTime;
        float dw =      (float) derivatives[5]  * Time.fixedDeltaTime;
        float dphi =    (float) derivatives[6]  * Time.fixedDeltaTime * Mathf.Rad2Deg;
        float dtheta =  (float) derivatives[7]  * Time.fixedDeltaTime * Mathf.Rad2Deg;
        float dpsi =    (float) derivatives[8]  * Time.fixedDeltaTime * Mathf.Rad2Deg;
        float dp =      (float) derivatives[9]  * Time.fixedDeltaTime * Mathf.Rad2Deg;
        float dq =      (float) derivatives[10] * Time.fixedDeltaTime * Mathf.Rad2Deg;
        float dr =      (float) derivatives[11] * Time.fixedDeltaTime * Mathf.Rad2Deg;
        
        // Update internal state
        position += new Vector3(dx, dy, dz);
        velocity += new Vector3(du, dv, dw);
        eulerAngles += new Vector3(dphi, dtheta, dpsi);
        angularVelocity += new Vector3(dp, dq, dr);
        
        // Alter drone GO position & rotation
        go.transform.position = new Vector3(position.x, position.z, -position.y);
        go.transform.eulerAngles = new Vector3(-eulerAngles.x, eulerAngles.z, eulerAngles.y);

        // Update Camera Pos
        Camera.transform.position = new Vector3(position.x - 1.83f, position.z + 0.64f, -position.y);
    }

    // Actions
    private void ThrottlePressed(InputAction.CallbackContext e)
    {
        Ut = UtVal;
    }
    private void ThrottleReleased(InputAction.CallbackContext e)
    {
        Ut = 0f;
    }
    
    private void MotorsPressed(InputAction.CallbackContext e)
    {
        URotorBase = UiVal;
    }
    private void MotorsReleased(InputAction.CallbackContext e)
    {
        URotorBase = 0f;
    }
}
