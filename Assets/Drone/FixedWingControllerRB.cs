using UnityEngine;
using UnityEngine.InputSystem;

public class FixedWingControllerRB : MonoBehaviour
{
    [SerializeField]
    public InputActionReference throttleIA, aileronIA, elevatorIA, rudderIA;
    public GameObject go;
    public GameObject Camera;
    public Rigidbody rb;

    [ReadOnly]
    public float Ut;
    [ReadOnly]
    public float Ua;
    [ReadOnly]
    public float Ue;
    [ReadOnly]
    public float Ur;

    
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
        Ue = 0f;
        Ur = 0f;

        go.transform.position = new Vector3(0, 0, 0);

        rb.velocity = new Vector3(0, 0, 0);

        go.transform.rotation = new()
        {
            eulerAngles = new Vector3(0, 0, 0)
        };

        rb.angularVelocity = new Vector3(0, 0, 0);
    }

    private void FixedUpdate()
    {        
        float newUe = Ue + elevatorIA.action.ReadValue<float>();
        Ue = Mathf.Clamp(newUe, -UeMinMax, UeMinMax);
        
        float newUa = aileronIA.action.ReadValue<float>();
        Ua = Mathf.Clamp(newUa, -UaMinMax, UaMinMax);

        float newUr = Ur + rudderIA.action.ReadValue<float>();
        Ur = Mathf.Clamp(newUr, -UrMinMax, UrMinMax);
    
        Vector3 position = go.transform.position;
        Vector3 velocity = rb.velocity;
        Vector3 eulerAngles = go.transform.rotation.eulerAngles;
        Vector3 angularVelocity = rb.angularVelocity;
        
        float x =       position.x;
        float y =       position.z;
        float z =       position.y;
        float u =       velocity.x;
        float v =       velocity.z;
        float w =       velocity.y;
        float phi =     eulerAngles.x     * Mathf.Deg2Rad;
        float theta =   -eulerAngles.z     * Mathf.Deg2Rad;
        float psi =     eulerAngles.y     * Mathf.Deg2Rad;
        float p =       angularVelocity.x;
        float q =       -angularVelocity.z;
        float r =       angularVelocity.y;
                
        FixedWingDynamicsAP dynamics = new(x, y, z, u, v, w, phi, theta, psi, p, q, r, Ua, Ue, Ur, Ut);

        // derivatives = {dx, dy, dz, du, dv, dw, dphi, dtheta, dpsi, dp, dq, dr}
        double[] derivatives = dynamics.GetDerivatives();
        float dx =      (float) derivatives[0] ;
        float dy =      (float) derivatives[1] ;
        float dz =      (float) derivatives[2] ;
        float du =      (float) derivatives[3] ;
        float dv =      (float) derivatives[4] ;
        float dw =      (float) derivatives[5] ;
        float dphi =    (float) derivatives[6] ;
        float dtheta =  (float) derivatives[7] ;
        float dpsi =    (float) derivatives[8] ;
        float dp =      (float) derivatives[9] ;
        float dq =      (float) derivatives[10];
        float dr =      (float) derivatives[11];
        

        rb.velocity = new Vector3(dx, dz, dy);
        rb.AddForce(new Vector3(du, dw, dv), ForceMode.VelocityChange);

        rb.angularVelocity = new Vector3(dphi, dpsi, -dtheta);
        rb.AddTorque(new Vector3(dp, dr, -dq), ForceMode.VelocityChange);

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
