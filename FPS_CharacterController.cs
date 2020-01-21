using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class FPS_CharacterController : MonoBehaviour, IAxisInput
{
    [Header("Camera")]
    public float minPitch = -80, maxPitch = 80;
    public Transform head;

    [Header("Collider")]
    public float height = 1.8f;
    public float radius = 0.25f;

    [Header("Physics")]
    public float drag = 0.1f;
    public float mass = 25f;
    public float maxInAirAcceleration = 0.1f;
    public int maxCollisions = 10;

    [Header("Kinematics")]
    public float forwardSpeed = 10;
    public float strafeSpeed = 7;
    public float backwardSpeed = 5;

    [Header("Ground Snapping")]
    public float probeDistance = 1f;
    public float maxSnapSpeed = 15f;
    public LayerMask probeMask = -1;

    Vector3 _MoveVector;
    Vector3 _AimVector;

    Vector3 _Velocity;
    Vector3 _Force;
    Vector3 _Acceleration;

    public bool Grounded { get; private set; }
    int stepsSinceLastGrounded;
    int groundSnapEnableTimer = 0;

    
    float minGroundDotProduct = 0.2f;
    Vector3 contactNormal;

    bool groundTransformChanged = true;
    Transform groundTransform;
    Vector3 ground_lsp_Pos;

    new CapsuleCollider collider;
    Collider[] contacts;

    Vector3 gravityOverride;
    bool UseCustomGravity;

    public Action<bool> OnGroundStateChanged;
    
    void Awake()
    {
        SetUpCollider();
    }

    void SetUpCollider()
    {
        collider = gameObject.AddComponent<CapsuleCollider>();
        collider.height = height - 0.25f;
        collider.radius = radius;
        collider.center = Vector3.up * Mathf.Lerp(0.25f, height, 0.5f);
        contacts = new Collider[maxCollisions];
    }

    void Update()
    {
        HandleRotation();

        if(!groundTransformChanged && Grounded)
        {
            transform.position = groundTransform.TransformPoint(ground_lsp_Pos);
        }

        stepsSinceLastGrounded++;
        if(groundSnapEnableTimer == 0 && (GroundTest() || SnapToGround()))
        {
            stepsSinceLastGrounded = 0;

            if(!Grounded && OnGroundStateChanged != null)
            {
                OnGroundStateChanged(true);
            }

            Grounded = true;
            _Velocity = WorldSpaceMoveVector(true);
            transform.position += _Velocity * Time.deltaTime;
            _Velocity = _Velocity * 0.5f;
        }
        else
        {
            if(Grounded && OnGroundStateChanged != null)
            {
                OnGroundStateChanged(false);
            }

            Grounded = false;
            PhysicsStep(Time.deltaTime);
        }

        CollisionTest();

        if(Grounded &! groundTransformChanged)
        {
            ground_lsp_Pos = groundTransform.InverseTransformPoint(transform.position);
        }

        if (groundSnapEnableTimer > 0)
        {
            groundSnapEnableTimer--;
        }
    }

    Vector3 WorldSpaceMoveVector(bool UseGroundNormal)
    {
        Vector3 zDirection = transform.forward;
        Vector3 xDirection = transform.right;
        if (UseGroundNormal)
        {
            zDirection = ProjectOnGroundPlane(zDirection).normalized;
            xDirection = ProjectOnGroundPlane(xDirection).normalized;
        }

        Vector3 mv = zDirection * _MoveVector.z * ((_MoveVector.z > 0) ? forwardSpeed : backwardSpeed);
        mv += xDirection * _MoveVector.x*strafeSpeed;

        return mv;
    }

    Vector3 ProjectOnGroundPlane(Vector3 v)
    {
        return v - contactNormal * Vector3.Dot(v, contactNormal);
    }

    void CollisionTest()
    {
        int hits = Physics.OverlapCapsuleNonAlloc(transform.position + transform.up * 0.25f, transform.position + transform.up * height, radius, contacts);
        for (int i = 0; i < hits; i++)
        {
            if(contacts[i] == collider)
            {
                continue;
            }

            if (Physics.ComputePenetration(collider, transform.position, transform.rotation, contacts[i], contacts[i].transform.position, contacts[i].transform.rotation, out Vector3 direction, out float distance))
            {
                Vector3 penetration = direction * distance;
                Vector3 velocityProjected = Vector3.Project(_Velocity, -direction);
                transform.position = transform.position + penetration;
                _Velocity += velocityProjected;
            }
        }
    }

    bool GroundTest()
    {
        if (Physics.Raycast(transform.position + transform.up, -transform.up, out RaycastHit hit, 1f))
        {
            contactNormal = hit.normal;
            if (groundTransform != hit.transform)
            {
                groundTransform = hit.transform;
                groundTransformChanged = true;
            }
            else
            {
                groundTransformChanged = false;
            }

            return true;
        }
        else
        {
            groundTransformChanged = true;
            return false;
        }
    }

    bool SnapToGround()
    {
        if(stepsSinceLastGrounded > 1)
        {
            return false;
        }
        float speed = _Velocity.magnitude;

        if(speed > maxSnapSpeed)
        {
            return false;
        }

        if (!Physics.Raycast(transform.position, -transform.up, out RaycastHit hit, probeDistance, probeMask))
        {
            return false;
        }
        if (hit.normal.y < minGroundDotProduct)
        {
            return false;
        }

        contactNormal = hit.normal;
        float dot = Vector3.Dot(_Velocity, hit.normal);
        if(dot > 0f)
        {
            _Velocity = (_Velocity - hit.normal * dot).normalized * speed;
        }
        transform.position = hit.point;
        return true;
    }

    void HandleRotation()
    {
        if(head == null)
        {
            return;
        }

        transform.localEulerAngles = _AimVector.y * Vector3.up;
        head.transform.localEulerAngles = _AimVector.x * Vector3.right;
    }

    public void AddMovementInput(Vector3 v)
    {
        _MoveVector = Vector3.ClampMagnitude(v, 1);
    }

    public void AddAimInput(Vector3 v)
    {
        _AimVector += v;
        _AimVector.x = Mathf.Clamp(_AimVector.x, minPitch, maxPitch);
    }

    public void AddForce(Vector3 force)
    {
        _Force = _Force + force;
        DisableGroundTest(10);
    }

    public void SetCustomGravity(Vector3 newGravity)
    {
        UseCustomGravity = true;
        gravityOverride = newGravity;
    }

    public void DisableCustomGravity()
    {
        UseCustomGravity = false;
    }

    public void DisableGroundTest(int frames = -1)
    {
        groundSnapEnableTimer = frames;
    }

    public void EnableGroundTest()
    {
        groundSnapEnableTimer = 0;
    }

    Vector3 Combine_Forces()
    {
        Vector3 dragForce = 0.5f * drag * (_Velocity * _Velocity.magnitude);
        Vector3 dragAccel = dragForce / mass;
        Vector3 gravAcc = (UseCustomGravity) ? gravityOverride : Physics.gravity;
        return gravAcc - dragAccel;
    }

    void PhysicsStep(float dt)
    {
        Vector3 n_Pos = transform.position + _Velocity * dt + _Acceleration * (dt * dt * 0.5f);
        Vector3 n_Acc = Combine_Forces() + _Force/mass;
        n_Acc = Vector3.MoveTowards(n_Acc, WorldSpaceMoveVector(false) + Combine_Forces(), maxInAirAcceleration);
        Vector3 n_Vel = _Velocity + (_Acceleration + n_Acc) * (dt * 0.5f);

        transform.position = n_Pos;
        _Velocity = n_Vel;
        _Acceleration = n_Acc;

        _Force = Vector3.zero;
    }
}
