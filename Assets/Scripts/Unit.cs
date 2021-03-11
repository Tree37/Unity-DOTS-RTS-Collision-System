using System;
using System.Runtime.InteropServices;
using Unity.Entities;
using UnityEngine;
using Unity.Transforms;
using Unity.Mathematics;

// same

public struct UnitTag : IComponentData
{

}

public struct HumanTag: IComponentData
{
}

public struct CavalryTag: IComponentData
{
}

public struct BeastTag : IComponentData
{
}

public struct Moving : IComponentData
{
    public int Value;
}
public struct Selected : IComponentData
{
    public bool Value;
}
public struct Direction : IComponentData
{
    public float2 Value;
}
public struct TargetPosition : IComponentData
{
    public float2 Value;
}
public struct MoveForce : IComponentData
{
    public float Value;
}
[StructLayout( LayoutKind.Explicit )]
public struct Velocity : IComponentData
{
    [FieldOffset(4)]
    public float3 Value;
}
public struct Drag : IComponentData
{
    public float Value;
}
public struct Mass : IComponentData
{
    public float Value;
}
public struct Radius : IComponentData
{
    public float Value;
}
public struct CollisionCell : IComponentData
{
    public int Value;
}
public struct CollisionCellMulti : IComponentData
{
    public int bL;
    public int bR;
    public int tL;
    public int tR;
}