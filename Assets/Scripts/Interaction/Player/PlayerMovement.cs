using UnityEngine;
using Unity.Mathematics;
using System;
using WorldConfig;
using WorldConfig.Gameplay.Player;
using WorldConfig.Generation.Entity;
namespace WorldConfig.Gameplay.Player{
    /// <summary>
    /// A collection of settings that describe how the player moves.
    /// Movement settings may change during gameplay.
    /// </summary>
    [Serializable]
    public class Movement : ICloneable{
        /// <summary> The maximum speed the player can walk at, in terms of world space. </summary>
        public float walkSpeed = 10f;
        /// <summary> The maximum speed the player can run at, in terms of world space. </summary>
        public float runSpeed = 15f;
        /// <summary>  How much force is applied to the player when they jump, in terms of world space. </summary>
        public float jumpForce = 8f;
        /// <summary> How much speed the user gains when moving, in terms of world space. The acceleration is added onto velocity
        /// meaning the comparative strength of friction increases with velocity. </summary>
        public float acceleration = 50f;
        /// <summary> How far below the player the ground needs to be for the player to be 'on the ground'. 
        /// Being on the ground may affect their ability to jump and the friction they experience. </summary>
        public float groundStickDist = 0.05f;
        /// <summary> A multiplier applied to all movement limits when the player is flying. For example, a multiplier of 2
        /// will mean the player will fly twice as fast as they can run </summary>
        public float flightSpeedMultiplier = 2f;
        /// <summary> Controls how the camera rotates and responds to user input. See <see cref="Gameplay.CameraMovement"/> for more information </summary>

        public object Clone(){
            return new Movement{
                walkSpeed = this.walkSpeed,
                runSpeed = this.runSpeed,
                jumpForce = this.jumpForce,
                acceleration = this.acceleration,
                groundStickDist = this.groundStickDist,
                flightSpeedMultiplier = this.flightSpeedMultiplier,
            };
        }
    }
}
public class PlayerMovement
{
    public static WorldConfig.Gameplay.Player.Camera Camera => Config.CURRENT.GamePlay.Player.value.Camera;
    public static WorldConfig.Gameplay.Player.Movement Setting => Config.CURRENT.GamePlay.Player.value.movement;
    private PlayerCamera cameraInput;
    private ref PlayerStreamer.Player data => ref PlayerHandler.data;

    public PlayerMovement(){
        //These constructors will hook themselves to input modules and will not be garbage collected
        new SurfaceMovement();
        new FlightMovement();
        cameraInput = new PlayerCamera(data);
    }

    public void Update(){ 
        cameraInput.LookRotation(ref data);
        InputPoller.InvokeStackTop("Movement::Update");
    }
}

public class MovementModule{
    public static WorldConfig.Gameplay.Player.Movement Setting => Config.CURRENT.GamePlay.Player.value.movement;
    public static ref float3 velocity => ref PlayerHandler.data.collider.velocity;
    public float2 InputDir;
    public virtual void Update(){}
}

public class SurfaceMovement : MovementModule{
    private float moveSpeed => IsSprinting ? Setting.runSpeed : Setting.walkSpeed;
    private bool IsSprinting;
    public SurfaceMovement()
    {
        InputPoller.AddStackPoll(new InputPoller.ActionBind("GroundMove::1", _ => Update()), "Movement::Update");
        InputPoller.AddStackPoll(new InputPoller.ActionBind("GroundMove::2", _ => PlayerHandler.data.collider.useGravity = true), "Movement::Gravity");
        InputPoller.AddBinding(new InputPoller.ActionBind("Move Vertical", (float y) => InputDir.y = y), "4.0::Movement");
        InputPoller.AddBinding(new InputPoller.ActionBind("Move Horizontal", (float x) => InputDir.x = x), "4.0::Movement");
        InputPoller.AddBinding(new InputPoller.ActionBind("Sprint", (float x) => {IsSprinting = true;}), "4.0::Movement");
        InputPoller.AddBinding(new InputPoller.ActionBind("Jump", (_null_) => {
            TerrainColliderJob.Settings collider = PlayerHandler.data.settings.collider;
            float3 posGS = PlayerHandler.data.position + collider.offset;
            if(PlayerHandler.data.collider.SampleCollision(posGS, new float3(collider.size.x, -Setting.groundStickDist, collider.size.z), out _))
                velocity += Setting.jumpForce * (float3)Vector3.up;
        }), "4.0::Movement");
    }

    public override void Update(){ 
        float2 desiredMove = ((float3)(PlayerHandler.camera.forward*InputDir.y + PlayerHandler.camera.right*InputDir.x)).xz;
        float2 deltaV = Setting.acceleration * Time.deltaTime * desiredMove;

        if(math.length(velocity.xz) < moveSpeed) 
            velocity.xz += deltaV;

        if(math.length(deltaV) > 0.1f){
            if(IsSprinting) PlayerHandler.data.animator.SetTrigger("IsRunning");
            else PlayerHandler.data.animator.SetTrigger("IsWalking");
        };

        IsSprinting = false;
        InputDir = float2.zero;
    }
}

public class FlightMovement : MovementModule{
    private float MoveSpeed => Setting.flightSpeedMultiplier * (IsSprinting ? Setting.runSpeed : Setting.walkSpeed);
    private int[] KeyBinds = null;
    private bool IsSprinting;

    public FlightMovement()
    {
        KeyBinds = null;
        InputPoller.AddBinding(new InputPoller.ActionBind("ToggleFly", (_null_) => {
            if(KeyBinds == null) AddHandles();
            else RemoveHandles();
        }), "4.0::Movement");
    }

    private void AddHandles(){
        InputPoller.AddStackPoll(new InputPoller.ActionBind("FlightMove::1", _ => Update()), "Movement::Update");
        InputPoller.AddStackPoll(new InputPoller.ActionBind("FlightMove::2", _ => PlayerHandler.data.collider.useGravity = false), "Movement::Gravity");
        InputPoller.AddKeyBindChange(() => {
            KeyBinds = InputPoller.KeyBindSaver.Rent(5);
            KeyBinds[0] = (int)InputPoller.AddBinding(new InputPoller.ActionBind("Ascend", (_null_) => {
                if(velocity.y < MoveSpeed) velocity.y += Setting.acceleration * Time.deltaTime * Setting.flightSpeedMultiplier;
            }), "4.0::Movement");
            KeyBinds[1] = (int)InputPoller.AddBinding(new InputPoller.ActionBind("Descend", (_null_) => {
                if(velocity.y > -MoveSpeed) velocity.y -= Setting.acceleration * Time.deltaTime * Setting.flightSpeedMultiplier;
            }), "4.0::Movement");
            KeyBinds[2] = (int)InputPoller.AddBinding(new InputPoller.ActionBind("Sprint", (float x) => {IsSprinting = true;}, InputPoller.ActionBind.Exclusion.ExcludeLayer), "4.0::Movement");
            KeyBinds[3] = (int)InputPoller.AddBinding(new InputPoller.ActionBind("Move Vertical",(float y) => InputDir.y = y, InputPoller.ActionBind.Exclusion.ExcludeLayer), "4.0::Movement");
            KeyBinds[4] = (int)InputPoller.AddBinding(new InputPoller.ActionBind("Move Horizontal", (float x) => InputDir.x = x, InputPoller.ActionBind.Exclusion.ExcludeLayer), "4.0::Movement");
        });
    }

    private void RemoveHandles(){
        InputPoller.RemoveStackPoll("FlightMove::1", "Movement::Update");
        InputPoller.RemoveStackPoll("FlightMove::2", "Movement::Gravity");
        InputPoller.AddKeyBindChange(() => {
            InputPoller.RemoveKeyBind((uint)KeyBinds[0], "4.0::Movement");
            InputPoller.RemoveKeyBind((uint)KeyBinds[1], "4.0::Movement");
            InputPoller.RemoveKeyBind((uint)KeyBinds[2], "4.0::Movement");
            InputPoller.RemoveKeyBind((uint)KeyBinds[3], "4.0::Movement");
            InputPoller.RemoveKeyBind((uint)KeyBinds[4], "4.0::Movement");
            InputPoller.KeyBindSaver.Return(KeyBinds);
            KeyBinds = null;
        });
    }

    public override void Update(){
        float2 desiredMove = ((float3)(PlayerHandler.camera.forward*InputDir.y + PlayerHandler.camera.right*InputDir.x)).xz;
        float2 deltaV = Setting.acceleration * Time.deltaTime * desiredMove *  Setting.flightSpeedMultiplier;

        velocity.y *= 1 - PlayerHandler.data.settings.collider.friction;
        if(math.length(velocity.xz) < MoveSpeed) 
            velocity.xz += deltaV;

        if(math.length(deltaV) > 0.1f){
            if(IsSprinting) PlayerHandler.data.animator.SetTrigger("IsRunning");
            else PlayerHandler.data.animator.SetTrigger("IsWalking");
        };
        
        IsSprinting = false;
        InputDir = float2.zero;
    }
}
