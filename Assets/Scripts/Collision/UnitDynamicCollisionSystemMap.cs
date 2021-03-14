using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

public class UnitDynamicCollisionSystemMap : SystemBase
{
    private const float CELL_SIZE = 1f;
    private const int CELLS_ACROSS = 4000;
    private int frame = 0;

    private EntityQuery query;
    private NativeMultiHashMap<int , ushort> map;

    private struct CollidingPair
    {
        public ushort unit1;
        public ushort unit2;
        public float distance;
    }

    protected unsafe override void OnStartRunning()
    {
        base.OnStartRunning();
        SetupMap();
    }
    protected override void OnUpdate()
    {
        MapSystem();
    }
    protected override void OnDestroy()
    {
        CleanupMap();
        base.OnDestroy();
    }

    private void SetupMap()
    {
        query = GetEntityQuery( typeof( UnitTag ) );
        map = new NativeMultiHashMap<int , ushort>( query.CalculateEntityCount() * 2 , Allocator.Persistent );
    }
    private void CleanupMap()
    {
        map.Dispose();
    }
    private void MapSystem()
    {
        query = GetEntityQuery( typeof( UnitTag ) );
        int numUnits = query.CalculateEntityCount();

        var copyPositions = new NativeArray<float3>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var copyVelocities = new NativeArray<float3>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var copyMass = new NativeArray<byte>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var collisionFlags = new NativeBitArray( numUnits , Allocator.TempJob , NativeArrayOptions.ClearMemory );
        var collisionPairs = new NativeQueue<CollidingPair>( Allocator.TempJob );

        var handle = new JobHandle();

        if ( frame >= 15 )
        {
            handle = RebuildMap();
            frame = 0;
        }
        else
        {
            handle = Dependency;
            frame++;
        }

        var copyJob = new CopyJob
        {
            copyPosition = copyPositions ,
            copyVelocity = copyVelocities ,
            copyMass = copyMass ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            velocityHandle = GetComponentTypeHandle<Velocity>() ,
            massHandle = GetComponentTypeHandle<Mass>()
        };
        handle = copyJob.ScheduleParallel( query , 1 , handle );

        var findCollidingPairsJob = new FindCollidingPairsJobMap
        {
            CELL_SIZE = CELL_SIZE ,
            CELLS_ACROSS = CELLS_ACROSS ,
            RADIUS = 0.5f ,
            //AVOID_RADIUS = 0.6f ,
            copyPositions = copyPositions ,
            collidingPairs = collisionPairs.AsParallelWriter() ,
            collisionFlags = collisionFlags ,
            map = map
        };
        handle = findCollidingPairsJob.Schedule( numUnits , 128 , handle );
        handle.Complete();

        var foundPairs = collisionPairs.ToArray( Allocator.TempJob );

        var resolveCollisionsJob = new ResolveCollisionsJob()
        {
            RADIUS = 0.5f ,
            collisionPairs = foundPairs ,
            copyPositions = copyPositions ,
            copyVelocities = copyVelocities ,
            copyMass = copyMass
        };
        handle = resolveCollisionsJob.Schedule( foundPairs.Length , 128 , handle );

        var writeResultsToUnits = new WriteDataJob
        {
            copyPositions = copyPositions ,
            copyVelocities = copyVelocities ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            velocityHandle = GetComponentTypeHandle<Velocity>()
        };
        handle = writeResultsToUnits.ScheduleParallel( query , 1 , handle );

        var disposeHandle = copyPositions.Dispose( handle );
        disposeHandle = JobHandle.CombineDependencies( copyVelocities.Dispose( handle ) , disposeHandle );
        disposeHandle = JobHandle.CombineDependencies( copyMass.Dispose( handle ) , disposeHandle );
        disposeHandle = JobHandle.CombineDependencies( collisionFlags.Dispose( handle ) , disposeHandle );
        disposeHandle = JobHandle.CombineDependencies( collisionPairs.Dispose( handle ) , disposeHandle );
        disposeHandle = JobHandle.CombineDependencies( foundPairs.Dispose( handle ) , disposeHandle );

        Dependency = disposeHandle;
    }
    private JobHandle RebuildMap()
    {
        var clearMapJob = new ClearMapJob
        {
            map = map
        };
        var handle = clearMapJob.Schedule( Dependency );

        var fillMapJob = new FillMapJob
        {
            CELLS_ACROSS = CELLS_ACROSS ,
            CELL_SIZE = CELL_SIZE ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            map = map.AsParallelWriter()
        };
        handle = fillMapJob.ScheduleParallel( query , 1 , handle );

        return handle;
    }

    [BurstCompile]
    private struct ClearMapJob : IJob
    {
        public NativeMultiHashMap<int , ushort> map;

        public void Execute()
        {
            map.Clear();
        }
    }
    [BurstCompile]
    private struct FillMapJob : IJobEntityBatchWithIndex
    {
        [ReadOnly] public float CELL_SIZE;
        [ReadOnly] public int CELLS_ACROSS;
        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;
        public NativeMultiHashMap<int , ushort>.ParallelWriter map;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int indexOfFirstEntityInQuery )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float px = batchTranslation[ i ].Value.x;
                float pz = batchTranslation[ i ].Value.z;
                int cell = ( int ) ( math.floor( px / CELL_SIZE ) + math.floor( pz / CELL_SIZE ) * CELLS_ACROSS );

                map.Add( cell , ( ushort ) ( indexOfFirstEntityInQuery + i ) );
            }
        }
    }
    [BurstCompile]
    private struct CopyJob : IJobEntityBatchWithIndex
    {
        [NativeDisableParallelForRestriction] public NativeArray<float3> copyPosition;
        [NativeDisableParallelForRestriction] public NativeArray<float3> copyVelocity;
        [NativeDisableParallelForRestriction] public NativeArray<byte> copyMass;
        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;
        [ReadOnly] public ComponentTypeHandle<Velocity> velocityHandle;
        [ReadOnly] public ComponentTypeHandle<Mass> massHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int firstEntityInQueryIndex )
        {
            NativeArray<Translation> batchPosition = batchInChunk.GetNativeArray( translationHandle );
            NativeArray<Velocity> batchVelocity = batchInChunk.GetNativeArray( velocityHandle );
            NativeArray<Mass> batchMass = batchInChunk.GetNativeArray( massHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                copyPosition[ firstEntityInQueryIndex + i ] = batchPosition[ i ].Value;
                copyVelocity[ firstEntityInQueryIndex + i ] = batchVelocity[ i ].Value;
                copyMass[ firstEntityInQueryIndex + i ] = ( byte ) batchMass[ i ].Value;
            }
        }
    }
    [BurstCompile]
    private struct WriteDataJob : IJobEntityBatchWithIndex
    {
        [ReadOnly] public NativeArray<float3> copyPositions;
        [ReadOnly] public NativeArray<float3> copyVelocities;

        [NativeDisableParallelForRestriction] public ComponentTypeHandle<Translation> translationHandle;
        [NativeDisableParallelForRestriction] public ComponentTypeHandle<Velocity> velocityHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int indexOfFirstEntityInQuery )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );
            NativeArray<Velocity> batchVelocity = batchInChunk.GetNativeArray( velocityHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                batchTranslation[ i ] = new Translation { Value = copyPositions[ indexOfFirstEntityInQuery + i ] };
                batchVelocity[ i ] = new Velocity { Value = copyVelocities[ indexOfFirstEntityInQuery + i ] };
            }
        }
    }
    [BurstCompile]
    private struct FindCollidingPairsJobMap : IJobParallelFor
    {
        [ReadOnly] public float CELL_SIZE;
        [ReadOnly] public int CELLS_ACROSS;
        [ReadOnly] public float RADIUS;
        //[ReadOnly] public float AVOID_RADIUS;

        [ReadOnly] public NativeMultiHashMap<int , ushort> map;
        [ReadOnly] public NativeArray<float3> copyPositions;
        [NativeDisableParallelForRestriction] public NativeQueue<CollidingPair>.ParallelWriter collidingPairs;
        [NativeDisableParallelForRestriction] public NativeBitArray collisionFlags;

        public void Execute( int index )
        {
            collisionFlags.Set( index , true );

            float px = copyPositions[ index ].x;
            float pz = copyPositions[ index ].z;

            float xmin = px - RADIUS;
            float zmin = pz - RADIUS;
            float xmax = px + RADIUS;
            float zmax = pz + RADIUS;
            int bl = ( int ) ( math.floor( xmin / CELL_SIZE ) + math.floor( zmin / CELL_SIZE ) * CELLS_ACROSS );
            int br = ( int ) ( math.floor( xmax / CELL_SIZE ) + math.floor( zmin / CELL_SIZE ) * CELLS_ACROSS );
            int tl = ( int ) ( math.floor( xmin / CELL_SIZE ) + math.floor( zmax / CELL_SIZE ) * CELLS_ACROSS );
            int tr = ( int ) ( math.floor( xmax / CELL_SIZE ) + math.floor( zmax / CELL_SIZE ) * CELLS_ACROSS );

            FixedList128<int> newCells = new FixedList128<int>();
            newCells.Add( bl );

            if ( br != bl )
                newCells.Add( br );
            if ( tl != bl )
                newCells.Add( tl );
            if ( br != tl )
                newCells.Add( tr );

            /*FixedList512<ushort> ids = new FixedList512<ushort>();
            FixedList512<float> distances = new FixedList512<float>();*/

            for ( int i = 0; i < newCells.Length; i++ )
            {
                int gridIndex = newCells[ i ];

                if ( map.TryGetFirstValue( gridIndex , out ushort otherUnitIndex , out var iterator ) )
                {
                    do
                    {
                        if ( !collisionFlags.IsSet( otherUnitIndex ) )
                        {
                            float px2 = copyPositions[ otherUnitIndex ].x;
                            float pz2 = copyPositions[ otherUnitIndex ].z;
                            float distance = math.sqrt( ( px - px2 ) * ( px - px2 ) + ( pz - pz2 ) * ( pz - pz2 ) );

                            if ( distance < RADIUS )
                            {
                                collidingPairs.Enqueue( new CollidingPair
                                {
                                    unit1 = ( ushort ) index ,
                                    unit2 = ( ushort ) otherUnitIndex ,
                                    distance = distance
                                } );
                            }
                        }
                    }
                    while ( map.TryGetNextValue( out otherUnitIndex , ref iterator ) );
                }
            }
        }
    }
    [BurstCompile]
    private struct ResolveCollisionsJob : IJobParallelFor
    {
        [ReadOnly] public float RADIUS;
        [ReadOnly] public float AVOID_RADIUS;

        [ReadOnly] public NativeArray<CollidingPair> collisionPairs;
        [NativeDisableParallelForRestriction] public NativeArray<float3> copyPositions;
        [NativeDisableParallelForRestriction] public NativeArray<float3> copyVelocities;
        [NativeDisableParallelForRestriction] public NativeArray<byte> copyMass;

        public void Execute( int index )
        {
            int unit1 = collisionPairs[ index ].unit1;
            int unit2 = collisionPairs[ index ].unit2;
            float distance = collisionPairs[ index ].distance;

            float px = copyPositions[ unit1 ].x;
            float py = copyPositions[ unit1 ].y;
            float pz = copyPositions[ unit1 ].z;
            float vx = copyVelocities[ unit1 ].x;
            float vy = copyVelocities[ unit1 ].y;
            float vz = copyVelocities[ unit1 ].z;
            float m = copyMass[ unit1 ];

            float px2 = copyPositions[ unit2 ].x;
            float py2 = copyPositions[ unit2 ].y;
            float pz2 = copyPositions[ unit2 ].z;
            float vx2 = copyVelocities[ unit2 ].x;
            float vy2 = copyVelocities[ unit2 ].y;
            float vz2 = copyVelocities[ unit2 ].z;
            float m2 = copyMass[ unit2 ];

            float overlap = 0.5f * ( distance - RADIUS );

            if ( distance > RADIUS )
            {
                overlap = 0.1f * ( distance - AVOID_RADIUS );
            }

            float ax = ( overlap * ( px - px2 ) ) / ( distance + 0.001f ); // math.clamp( ( overlap * ( px - px2 ) ) / ( distance + 0.001f ) , -1 , 1 );
            float az = ( overlap * ( pz - pz2 ) ) / ( distance + 0.001f ); //math.clamp( ( overlap * ( pz - pz2 ) ) / ( distance + 0.001f ) , -1 , 1 );
            ax = math.clamp( ax , -1f , 1f );
            az = math.clamp( az , -1f , 1f );

            /*if ( overlap >= 0 )
            {
                ax = 0.001f;
                az = 0.001f;
            }*/

            float ax1 = px - ax;
            float az1 = pz - az;
            float ax2 = px2 + ax;
            float az2 = pz2 + az;

            float2 pos1 = new float2( px , pz );
            float2 pos2 = new float2( px2 , pz2 );
            float2 vel1 = new float2( vx , vz );
            float2 vel2 = new float2( vx2 , vz2 );
            float2 normal = math.normalizesafe( pos1 - pos2 );
            float a1 = math.dot( vel1 , normal );
            float a2 = math.dot( vel2 , normal );
            float pe = ( 2 * ( a1 - a2 ) ) / ( m + m2 );
            float2 nVel1 = vel1 - pe * m2 * normal;
            float2 nVel2 = vel2 + pe * m2 * normal;

            copyPositions[ unit1 ] = new float3( ax1 , py , az1 );
            copyVelocities[ unit1 ] = new float3( vx , vy , vz );
            copyPositions[ unit2 ] = new float3( ax2 , py2 , az2 );
            copyVelocities[ unit2 ] = new float3( vx2 , vy2 , vz2 );

            /*copyPositions[ unit1 ] = new float3( ax1 , py , az1 );
            copyVelocities[ unit1 ] = new float3( nVel1.x , vy , nVel1.y );
            copyPositions[ unit2 ] = new float3( ax2 , py2 , az2 );
            copyVelocities[ unit2 ] = new float3( nVel2.x , vy2 , nVel2.y );*/
        }
    }
}
