using UnityEngine;
using UnityEngine.InputSystem;

public class FixedWingController : MonoBehaviour
{
    [SerializeField]
    public InputActionReference throttleIA, aileronIA, elevatorIA, rudderIA;
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


    private void Awake() {
        throttleIA.action.performed += ThrottlePressed;
        throttleIA.action.canceled  += ThrottleReleased;
    }
    
    // Start is called before the first frame update
    private void Start()
    {
        // Setup initial condition
        Ut = 0;
        Ua = 0f;
        Ue = 0.2f;
        Ur = 0f;

        position = new Vector3(0, 0, 0);
        go.transform.position = position;

        velocity = new Vector3(60, 0, 0);

        eulerAngles = new Vector3(0, 0, 0);
        rotation.eulerAngles = eulerAngles;
        go.transform.rotation = rotation;

        angularVelocity = new Vector3(0, 0, 0);
    }

    private void FixedUpdate()
    {        
        float newUe = Ue + elevatorIA.action.ReadValue<float>();
        Ue = Mathf.Clamp(newUe, -UeMinMax, UeMinMax);
        
        float newUa = aileronIA.action.ReadValue<float>();
        Ua = Mathf.Clamp(newUa, -UaMinMax, UaMinMax);

        float newUr = Ur + rudderIA.action.ReadValue<float>();
        Ur = Mathf.Clamp(newUr, -UrMinMax, UrMinMax);
        
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
                
        FixedWingDynamicsAP dynamics = new(x, y, z, u, v, w, phi, theta, psi, p, q, r, Ua, Ue, Ur, Ut);

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
        
        position += new Vector3(dx, dy, dz);
        go.transform.position = new Vector3(position.x, position.z, position.y);
        
        velocity += new Vector3(du, dv, dw);

        eulerAngles += new Vector3(dphi, dtheta, dpsi);
        rotation.eulerAngles = new Vector3(eulerAngles.x, eulerAngles.z, -eulerAngles.y);
        go.transform.rotation = rotation;
        
        angularVelocity += new Vector3(dp, dq, dr);

        // Update Camera Pos
        Camera.transform.position = new Vector3(x + dx - 1.83f, z + dz + 0.64f, y + dy);
    }

    // Actions
    private void ThrottlePressed(InputAction.CallbackContext e)
    {
        Ut = 0.05f;
    }
    private void ThrottleReleased(InputAction.CallbackContext e)
    {
        Ut = 0f;
    }
}
