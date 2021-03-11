using Unity.Entities;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

// reworked

[DisableAutoCreation]
public class UnitMovementSystem : SystemBase
{
    private int MAP_SIZE;
    private int MAP_SCALE;

    private EntityQuery query;

    public NativeArray<float3> terrainVertices;

    protected override void OnCreate()
    {
        base.OnCreate();
        var description = new EntityQueryDesc()
        {
            All = new ComponentType[] {
                ComponentType.ReadWrite<Translation>(),
                ComponentType.ReadOnly<Velocity>() }
        };
        query = GetEntityQuery( description );
    }
    protected override void OnUpdate()
    {
        query = GetEntityQuery( typeof( UnitTag ) );
        JobHandle handle = new JobHandle();

        var calculateDirectionJob = new CalculateDirectionJob
        {
            translationHandle = GetComponentTypeHandle<Translation>() ,
            targetHandle = GetComponentTypeHandle<TargetPosition>() ,
            directionHandle = GetComponentTypeHandle<Direction>()
        };
        handle = calculateDirectionJob.ScheduleParallel( query , 1 , Dependency );

        var updateMovingJob = new UpdateMovementStateJob
        {
            translationHandle = GetComponentTypeHandle<Translation>() ,
            targetHandle = GetComponentTypeHandle<TargetPosition>() ,
            movingHandle = GetComponentTypeHandle<Moving>()
        };
        handle = updateMovingJob.ScheduleParallel( query , 1 , handle );

        var applyMoveForceJob = new ApplyMoveForceJob
        {
            forceHandle = GetComponentTypeHandle<MoveForce>() ,
            directionHandle = GetComponentTypeHandle<Direction>() ,
            movingHandle = GetComponentTypeHandle<Moving>() ,
            velocityHandle = GetComponentTypeHandle<Velocity>()
        };
        handle = applyMoveForceJob.ScheduleParallel( query , 1 , handle );

        var applyDragJob = new ApplyDragJob
        {
            dragHandle = GetComponentTypeHandle<Drag>() ,
            velocityHandle = GetComponentTypeHandle<Velocity>()
        };
        handle = applyDragJob.ScheduleParallel( query , 1 , handle );

        var applyVelocityJob = new ApplyVelocityJob
        {
            dt = Time.DeltaTime ,
            velocityHandle = GetComponentTypeHandle<Velocity>() ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
        };
        handle = applyVelocityJob.ScheduleParallel( query , 1 , handle );

        var boundsJob = new KeepWithinBoundsJob
        {
            translationHandle = GetComponentTypeHandle<Translation>()
        };
        handle = boundsJob.ScheduleParallel( query , 1 , handle );

        /*var heightsJob = new SetHeightsJob2
        {
            HALF_HEIGHT = 0.5f ,
            MAP_SIZE = MAP_SIZE ,
            MAP_SCALE = MAP_SCALE ,
            vertices = terrainVertices ,
            translationHandle = GetComponentTypeHandle<Translation>()
        };
        handle = heightsJob.ScheduleParallel( query , 1 , handle );*/

        Dependency = handle;
    }
    protected override void OnDestroy()
    {
        terrainVertices.Dispose();
        base.OnDestroy();
    }

    public void SetVertices( Vector3[] _vertices , int mapSize , int mapScale )
    {
        MAP_SIZE = mapSize;
        MAP_SCALE = mapScale;

        terrainVertices = new NativeArray<float3>( _vertices.Length , Allocator.Persistent );

        for ( int i = 0; i < _vertices.Length; i++ )
        {
            terrainVertices[ i ] = _vertices[ i ];
        }
    }

    [BurstCompile]
    private struct CalculateDirectionJob : IJobEntityBatch
    {
        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;
        [ReadOnly] public ComponentTypeHandle<TargetPosition> targetHandle;
        public ComponentTypeHandle<Direction> directionHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );
            NativeArray<TargetPosition> batchTarget = batchInChunk.GetNativeArray( targetHandle );
            NativeArray<Direction> batchDirection = batchInChunk.GetNativeArray( directionHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float2 pos2D = new float2( batchTranslation[ i ].Value.x , batchTranslation[ i ].Value.z );
                float distance = math.distance( batchTarget[ i ].Value , pos2D );
                float2 direction = batchTarget[ i ].Value - pos2D;
                float2 directionNormalized = direction / distance;
                batchDirection[ i ] = new Direction { Value = directionNormalized };
            }
        }
    }
    [BurstCompile]
    private struct UpdateMovementStateJob : IJobEntityBatch
    {
        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;
        [ReadOnly] public ComponentTypeHandle<TargetPosition> targetHandle;
        public ComponentTypeHandle<Moving> movingHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );
            NativeArray<TargetPosition> batchTarget = batchInChunk.GetNativeArray( targetHandle );
            NativeArray<Moving> batchMoving = batchInChunk.GetNativeArray( movingHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float2 pos2D = new float2( batchTranslation[ i ].Value.x , batchTranslation[ i ].Value.z );
                float distance = math.distance( batchTarget[ i ].Value , pos2D );
                int moving = math.select( 1 , -1 , distance <= 0.05f );
                batchMoving[ i ] = new Moving { Value = moving };
            }
        }
    }
    [BurstCompile]
    private struct ApplyMoveForceJob : IJobEntityBatch
    {
        [ReadOnly] public ComponentTypeHandle<MoveForce> forceHandle;
        [ReadOnly] public ComponentTypeHandle<Direction> directionHandle;
        [ReadOnly] public ComponentTypeHandle<Moving> movingHandle;
        public ComponentTypeHandle<Velocity> velocityHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<MoveForce> batchForce = batchInChunk.GetNativeArray( forceHandle );
            NativeArray<Direction> batchDirection = batchInChunk.GetNativeArray( directionHandle );
            NativeArray<Moving> batchMoving = batchInChunk.GetNativeArray( movingHandle );
            NativeArray<Velocity> batchVelocity = batchInChunk.GetNativeArray( velocityHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                if ( batchMoving[ i ].Value == 1 )
                {
                    float3 direction3D = new float3( batchDirection[ i ].Value.x , 0 , batchDirection[ i ].Value.y );
                    float3 newVelocity = batchVelocity[ i ].Value + batchForce[ i ].Value * direction3D;
                    batchVelocity[ i ] = new Velocity { Value = newVelocity };
                }
            }
        }
    }
    [BurstCompile]
    private struct ApplyDragJob : IJobEntityBatch
    {
        [ReadOnly] public ComponentTypeHandle<Drag> dragHandle;
        public ComponentTypeHandle<Velocity> velocityHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Drag> batchDrag = batchInChunk.GetNativeArray( dragHandle );
            NativeArray<Velocity> batchVelocity = batchInChunk.GetNativeArray( velocityHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float3 drag = batchVelocity[ i ].Value / batchDrag[ i ].Value;
                float3 newVelocity = batchVelocity[ i ].Value - drag;
                batchVelocity[ i ] = new Velocity { Value = newVelocity };
            }
        }
    }
    [BurstCompile]
    private struct ApplyVelocityJob : IJobEntityBatch
    {
        [ReadOnly] public float dt;
        [ReadOnly] public ComponentTypeHandle<Velocity> velocityHandle;
        public ComponentTypeHandle<Translation> translationHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Velocity> batchVelocity = batchInChunk.GetNativeArray( velocityHandle );
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float3 newTranslation = batchTranslation[ i ].Value + batchVelocity[ i ].Value * dt;
                batchTranslation[ i ] = new Translation { Value = newTranslation };
            }
        }
    }
    [BurstCompile]
    private struct KeepWithinBoundsJob : IJobEntityBatch
    {
        public ComponentTypeHandle<Translation> translationHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float newX = batchTranslation[ i ].Value.x;
                float newZ = batchTranslation[ i ].Value.z;

                if ( newX <= 20 )
                {
                    newX = 21;
                }
                if ( newZ <= 20 )
                {
                    newZ = 21;
                }

                batchTranslation[ i ] = new Translation { Value = new float3( newX , batchTranslation[ i ].Value.y , newZ ) };
            }
        }
    }
    [BurstCompile]
    private struct SetHeightsJob : IJobEntityBatch
    {
        [ReadOnly] public float HALF_HEIGHT;
        [ReadOnly] public int MAP_SIZE;
        [ReadOnly] public int MAP_SCALE;
        [ReadOnly] public NativeArray<float3> vertices;

        public ComponentTypeHandle<Translation> translationHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float px = batchTranslation[ i ].Value.x;
                float pz = batchTranslation[ i ].Value.z;

                int index = ( int ) ( math.floor( pz / MAP_SCALE ) * MAP_SIZE + math.floor( px / MAP_SCALE ) );

                int bL = index;
                int bR = index + 1;
                int tL = index + MAP_SIZE;

                float3 p1 = vertices[ bL ];
                float3 p2 = vertices[ bR ];
                float3 p3 = vertices[ tL ];

                float3 normal = math.cross( ( p2 - p1 ) , ( p3 - p1 ) );

                float py = ( normal.x * ( p1.x - px ) + normal.z * ( p1.z - pz ) + normal.y * p1.y ) / normal.y + HALF_HEIGHT;

                batchTranslation[ i ] = new Translation { Value = new float3( px , py , pz ) };
            }
        }
    }
    [BurstCompile]
    private struct SetHeightsJob2 : IJobEntityBatch
    {
        [ReadOnly] public float HALF_HEIGHT;
        [ReadOnly] public int MAP_SIZE;
        [ReadOnly] public int MAP_SCALE;
        [ReadOnly] public NativeArray<float3> vertices;

        public ComponentTypeHandle<Translation> translationHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray<Translation>( translationHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float px = batchTranslation[ i ].Value.x;
                float pz = batchTranslation[ i ].Value.z;

                int index = ( int ) ( math.floor( pz / MAP_SCALE ) * MAP_SIZE + math.floor( px / MAP_SCALE ) );

                int bL = index;
                int bR = index + 1;
                int tL = index + MAP_SIZE;
                int tR = index + MAP_SIZE + 1;

                float3 V1 = vertices[ bL ];
                float3 V2 = vertices[ bR ];
                float3 V3 = vertices[ tL ];
                float3 V4 = vertices[ tR ];

                float py;

                if ( IsInside( V1.x , V1.z , V2.x , V2.z , V4.x , V4.z , px , pz ) )
                    py = CalcY( V1 , V2 , V4 , px , pz ) + HALF_HEIGHT;
                else
                    py = CalcY( V1 , V3 , V4 , px , pz ) + HALF_HEIGHT;

                batchTranslation[ i ] = new Translation { Value = new float3( px , py , pz ) };
            }
        }

        private double Area( double x1 , double z1 , double x2 , double z2 , double x3 , double z3 )
        {
            return math.abs( ( x1 * ( z2 - z3 ) + x2 * ( z3 - z1 ) + x3 * ( z1 - z2 ) ) / 2.0f );
        }

        private bool IsInside( double x1 , double y1 , double x2 , double y2 , double x3 , double y3 , double x , double y )
        {
            double A = Area( x1 , y1 , x2 , y2 , x3 , y3 ); // Calculate area of triangle ABC
            double A1 = Area( x , y , x2 , y2 , x3 , y3 ); // Calculate area of triangle PBC
            double A2 = Area( x1 , y1 , x , y , x3 , y3 ); // Calculate area of triangle PAC
            double A3 = Area( x1 , y1 , x2 , y2 , x , y ); // Calculate area of triangle PAB
            return ( A == A1 + A2 + A3 ); // Check if sum of A1, A2 and A3 is same as A
        }

        // Returns y coordinate of point on triangle given point x and z coordinates
        private float CalcY( double3 p1 , double3 p2 , double3 p3 , double x , double z )
        {
            // determinant
            double det = ( p2.z - p3.z ) * ( p1.x - p3.x ) + ( p3.x - p2.x ) * ( p1.z - p3.z );

            double l1 = ( ( p2.z - p3.z ) * ( x - p3.x ) + ( p3.x - p2.x ) * ( z - p3.z ) ) / det;
            double l2 = ( ( p3.z - p1.z ) * ( x - p3.x ) + ( p1.x - p3.x ) * ( z - p3.z ) ) / det;
            double l3 = 1.0f - l1 - l2;

            return ( float ) ( l1 * p1.y + l2 * p2.y + l3 * p3.y );
        }
    }

    // to incorporate upward and downward momentum:
    // check the height of the triangle one unit in the current direction,
    // if its above decrease move force
    // if its below decrease move force
    // problem solved!
    // we can just manually set heights with this
    // and collisions is decoupled!!!!
}