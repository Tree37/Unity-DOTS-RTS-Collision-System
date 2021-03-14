using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

// reworked

// CAPITOL VARIABLE NAMES WITH _ ARE CONSTANTS
public class UnitDynamicCollisionSystemGrid : SystemBase
{
    private NativeArray<ushort> personGrid;
    private NativeQueue<UpdateCellData> cellsToUpdate;

    private const float CELL_SIZE = 1f;
    private const int CELLS_ACROSS = 4000;
    private const int CELL_CAPACITY = 8;
    private const ushort VOID_CELL_VALUE = 60000;

    private EntityQuery query;

    protected unsafe override void OnStartRunning()
    {
        base.OnStartRunning();
        //Setup();
    }
    protected override void OnUpdate()
    {
        //CollisionSystemState();
    }
    protected override void OnDestroy()
    {
        //Cleanup();
        base.OnDestroy();
    }

    private void Setup()
    {
        personGrid = new NativeArray<ushort>( CELLS_ACROSS * CELLS_ACROSS * CELL_CAPACITY , Allocator.Persistent , NativeArrayOptions.UninitializedMemory );
        cellsToUpdate = new NativeQueue<UpdateCellData>( Allocator.Persistent );
        query = GetEntityQuery( typeof( UnitTag ) );

        InitializeGridJob initJob = new InitializeGridJob // set all grid values to void value
        {
            VOID_CELL_VALUE = VOID_CELL_VALUE ,
            spatialGrid = personGrid ,
        };
        JobHandle handle = initJob.Schedule( Dependency );
        handle.Complete();
        BuildGridJob buildGridJob = new BuildGridJob
        {
            CELL_SIZE = CELL_SIZE ,
            CELLS_ACROSS = CELLS_ACROSS ,
            CELL_CAPACITY = CELL_CAPACITY ,
            VOID_CELL_VALUE = VOID_CELL_VALUE ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            cellHandle = GetComponentTypeHandle<CollisionCell>() ,
            grid = personGrid ,
        };
        handle = buildGridJob.Schedule( query , handle );
        handle.Complete();
        Dependency = handle;
    }
    private void Cleanup()
    {
        personGrid.Dispose();
        cellsToUpdate.Dispose();
        base.OnDestroy();
    }
    private void CollisionSystemState()
    {
        query = GetEntityQuery( typeof( UnitTag ) );
        int numUnits = query.CalculateEntityCount();

        var copyPositions   = new NativeArray<float3>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var copyVelocities  = new NativeArray<float3>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var copyMass        = new NativeArray<byte>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var collisionFlags  = new NativeBitArray( numUnits , Allocator.TempJob , NativeArrayOptions.ClearMemory );
        var collisionPairs  = new NativeQueue<CollidingPair>( Allocator.TempJob );

        var updateUnitCells = new UpdateUnitCellsJob // Update the cell the unit is currently in and queue changed cells
        {
            CELL_SIZE = CELL_SIZE ,
            CELLS_ACROSS = CELLS_ACROSS ,
            unitsToUpdate = cellsToUpdate.AsParallelWriter() ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            cellHandle = GetComponentTypeHandle<CollisionCell>()
        };
        JobHandle handle = updateUnitCells.ScheduleParallel( query , 1 , Dependency );

        var updateGridCells = new UpdateGridCellsJob // remove units from old cells and add to new ones
        {
            CELL_CAPACITY = CELL_CAPACITY ,
            VOID_CELL_VALUE = VOID_CELL_VALUE ,
            grid = personGrid ,
            cellData = cellsToUpdate
        };
        handle = updateGridCells.Schedule( handle );

        var copyUnitPosition = new CopyPositionsJob
        {
            translationHandle = GetComponentTypeHandle<Translation>() ,
            copyArray = copyPositions
        };
        var copyUnitVelocity = new CopyVelocitesJob
        {
            velocityHandle = GetComponentTypeHandle<Velocity>() ,
            copyArray = copyVelocities
        };
        var copyUnitMass = new CopyMassJob
        {
            massHandle = GetComponentTypeHandle<Mass>() ,
            copyArray = copyMass
        };
        handle = JobHandle.CombineDependencies(
            copyUnitPosition.ScheduleParallel( query , 1 , handle ) ,
            copyUnitVelocity.ScheduleParallel( query , 1 , handle ) ,
            copyUnitMass.ScheduleParallel( query , 1 , handle ) );

        var findCollidingPairs = new FindCollidingPairsJob
        {
            CELL_SIZE = CELL_SIZE ,
            CELLS_ACROSS = CELLS_ACROSS ,
            CELL_CAPACITY = CELL_CAPACITY ,
            RADIUS = 0.5f ,
            AVOID_RADIUS = 0.6f ,
            grid = personGrid ,
            copyPositions = copyPositions ,
            collidingPairs = collisionPairs.AsParallelWriter() ,
            collisionFlags = collisionFlags ,
        };
        handle = findCollidingPairs.Schedule( copyPositions.Length , 64 , handle );
        handle.Complete();

        var foundPairs = collisionPairs.ToArray( Allocator.TempJob );

        var resolveCollisionsJob = new ResolveCollisionsJob()
        {
            RADIUS = 0.5f ,
            AVOID_RADIUS = 0.6f ,
            collisionPairs = foundPairs ,
            copyPositions = copyPositions ,
            copyVelocities = copyVelocities ,
            copyMass = copyMass
        };
        handle = resolveCollisionsJob.Schedule( foundPairs.Length , 64 , handle );

        var writeResultsToUnits = new WriteDataJob
        {
            copyPositions = copyPositions ,
            copyVelocities = copyVelocities ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            velocityHandle = GetComponentTypeHandle<Velocity>()
        };
        handle = writeResultsToUnits.ScheduleParallel( query , 1 , handle );

        var disposeHandle = copyPositions.Dispose( handle );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , copyVelocities.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , copyMass.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , collisionFlags.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , collisionPairs.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , foundPairs.Dispose( handle ) );

        Dependency = disposeHandle;
    }
    private void CollisionSystemStateLess()
    {
        query = GetEntityQuery( typeof( UnitTag ) );
        int numUnits = query.CalculateEntityCount();

        var copyPositions = new NativeArray<float3>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var copyVelocities = new NativeArray<float3>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var copyMass = new NativeArray<byte>( numUnits , Allocator.TempJob , NativeArrayOptions.UninitializedMemory );
        var collisionFlags = new NativeBitArray( numUnits , Allocator.TempJob , NativeArrayOptions.ClearMemory );
        var collisionPairs = new NativeQueue<CollidingPair>( Allocator.TempJob );

        var clearJob = new ClearGridJob
        {
            CELL_CAPACITY = CELL_CAPACITY ,
            VOID_CELL_VALUE = VOID_CELL_VALUE ,
            cellHandle = GetComponentTypeHandle<CollisionCell>() ,
            grid = personGrid
        };
        var handle = clearJob.ScheduleParallel( query , 1 , Dependency );

        var buildJob = new BuildGridJob
        {
            CELLS_ACROSS = CELLS_ACROSS ,
            CELL_CAPACITY = CELL_CAPACITY ,
            CELL_SIZE = CELL_SIZE ,
            VOID_CELL_VALUE = VOID_CELL_VALUE ,
            cellHandle = GetComponentTypeHandle<CollisionCell>() ,
            translationHandle = GetComponentTypeHandle<Translation>() ,
            grid = personGrid
        };
        var buildHandle = buildJob.Schedule( query , handle );

        var copyUnitPosition = new CopyPositionsJob
        {
            translationHandle = GetComponentTypeHandle<Translation>() ,
            copyArray = copyPositions
        };
        var copyUnitVelocity = new CopyVelocitesJob
        {
            velocityHandle = GetComponentTypeHandle<Velocity>() ,
            copyArray = copyVelocities
        };
        var copyUnitMass = new CopyMassJob
        {
            massHandle = GetComponentTypeHandle<Mass>() ,
            copyArray = copyMass
        };
        var copyHandle = JobHandle.CombineDependencies(
            copyUnitPosition.ScheduleParallel( query , 1 , handle ) ,
            copyUnitVelocity.ScheduleParallel( query , 1 , handle ) ,
            copyUnitMass.ScheduleParallel( query , 1 , handle ) );

        handle = JobHandle.CombineDependencies( buildHandle , copyHandle );

        var findCollidingPairs = new FindCollidingPairsJob
        {
            CELL_SIZE = CELL_SIZE ,
            CELLS_ACROSS = CELLS_ACROSS ,
            CELL_CAPACITY = CELL_CAPACITY ,
            RADIUS = 0.5f ,
            AVOID_RADIUS = 0.75f ,
            grid = personGrid ,
            copyPositions = copyPositions ,
            collidingPairs = collisionPairs.AsParallelWriter() ,
            collisionFlags = collisionFlags ,
        };
        handle = findCollidingPairs.Schedule( copyPositions.Length , 128 , handle );
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
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , copyVelocities.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , copyMass.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , collisionFlags.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , collisionPairs.Dispose( handle ) );
        disposeHandle = JobHandle.CombineDependencies( disposeHandle , foundPairs.Dispose( handle ) );

        Dependency = disposeHandle;
    }
    private void CollisionSystemStateLessHierachical()
    {
        
    }

    [BurstCompile]
    private unsafe struct InitializeGridJob : IJob
    {
        [ReadOnly] public ushort VOID_CELL_VALUE;
        public NativeArray<ushort> spatialGrid;

        public void Execute()
        {
            ushort value = VOID_CELL_VALUE;
            void* p = &value;
            UnsafeUtility.MemCpyReplicate( spatialGrid.GetUnsafePtr() , p , sizeof( ushort ) , spatialGrid.Length );
        }
    }    
    [BurstCompile]
    private struct BuildGridJob : IJobEntityBatchWithIndex
    {
        [ReadOnly] public float CELL_SIZE;
        [ReadOnly] public int CELLS_ACROSS;
        [ReadOnly] public int CELL_CAPACITY;
        [ReadOnly] public ushort VOID_CELL_VALUE;

        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;
        public ComponentTypeHandle<CollisionCell> cellHandle;
        public NativeArray<ushort> grid;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int indexOfFirstEntityInQuery )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );
            NativeArray<CollisionCell> batchCell = batchInChunk.GetNativeArray( cellHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float px = batchTranslation[ i ].Value.x;
                float pz = batchTranslation[ i ].Value.z;
                int cell = ( int ) ( math.floor( px / CELL_SIZE ) + math.floor( pz / CELL_SIZE ) * CELLS_ACROSS );
                int gridIndex = cell * CELL_CAPACITY;

                batchCell[ i ] = new CollisionCell { Value = cell };

                for ( int j = 0; j < CELL_CAPACITY; j++ )
                {
                    if ( grid[ gridIndex + j ] == VOID_CELL_VALUE )
                    {
                        grid[ gridIndex + j ] = ( ushort ) ( indexOfFirstEntityInQuery + i );
                        break;
                    }
                }
            }
        }
    }
    [BurstCompile]
    private struct ClearGridJob : IJobEntityBatch
    {
        [ReadOnly] public ushort VOID_CELL_VALUE;
        [ReadOnly] public int CELL_CAPACITY;
        [ReadOnly] public ComponentTypeHandle<CollisionCell> cellHandle;
        [NativeDisableParallelForRestriction] public NativeArray<ushort> grid;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex )
        {
            NativeArray<CollisionCell> batchCell = batchInChunk.GetNativeArray( cellHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                int gridIndex = batchCell[ i ].Value * CELL_CAPACITY;

                for ( int j = 0; j < CELL_CAPACITY; j++ )
                {
                    grid[ gridIndex + j ] = VOID_CELL_VALUE; 
                }
            }
        }
    }
    [BurstCompile]
    private struct UpdateUnitCellsJob : IJobEntityBatchWithIndex
    {
        [ReadOnly] public float CELL_SIZE;
        [ReadOnly] public int CELLS_ACROSS;
        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;
        public ComponentTypeHandle<CollisionCell> cellHandle;
        public NativeQueue<UpdateCellData>.ParallelWriter unitsToUpdate;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int indexOfFirstEntityInQuery )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );
            NativeArray<CollisionCell> batchCell = batchInChunk.GetNativeArray( cellHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                float px = batchTranslation[ i ].Value.x;
                float pz = batchTranslation[ i ].Value.z;
                int newCell = ( int ) ( math.floor( px / CELL_SIZE ) + math.floor( pz / CELL_SIZE ) * CELLS_ACROSS );
                int oldCell = batchCell[ i ].Value;

                batchCell[ i ] = new CollisionCell { Value = newCell };

                if ( oldCell != newCell )
                {
                    UpdateCellData data = new UpdateCellData
                    {
                        unitID = indexOfFirstEntityInQuery + i ,
                        oldCell = oldCell ,
                        newCell = newCell
                    };

                    unitsToUpdate.Enqueue( data );
                }
            }
        }
    }
    [BurstCompile]
    private struct UpdateGridCellsJob : IJob
    {
        [ReadOnly] public int CELL_CAPACITY;
        [ReadOnly] public ushort VOID_CELL_VALUE;

        public NativeArray<ushort> grid;
        public NativeQueue<UpdateCellData> cellData;

        public void Execute()
        {
            // this looks slow but because units rarely change cells every frame the job time is very small
            while ( cellData.TryDequeue( out UpdateCellData data ) )
            {
                int gridIndex = data.oldCell * CELL_CAPACITY;

                for ( int i = 0; i < CELL_CAPACITY; i++ )
                {
                    // basically loop through the cell until id is found
                    // then shift all following units down one
                    if ( grid[ gridIndex + i ] == data.unitID )
                    {
                        int shiftIndex = gridIndex + i;
                        int endOfCell = gridIndex + CELL_CAPACITY - 1;
                        while ( grid[ shiftIndex ] != VOID_CELL_VALUE && shiftIndex < endOfCell ) // remove the unit by shifting all following units down one
                        {
                            grid[ shiftIndex ] = grid[ shiftIndex + 1 ];
                            shiftIndex++;
                        }

                        grid[ shiftIndex ] = VOID_CELL_VALUE;
                        break;
                    }
                }

                // add the unit id to the new cell it occupies
                gridIndex = data.newCell * CELL_CAPACITY;

                for ( int i = 0; i < CELL_CAPACITY; i++ )
                {
                    if ( grid[ gridIndex + i ] == VOID_CELL_VALUE )
                    {
                        grid[ gridIndex + i ] = ( ushort ) data.unitID;
                        break;
                    }
                }
            }

            cellData.Clear();
        }
    }
    [BurstCompile]
    private struct CopyPositionsJob : IJobEntityBatchWithIndex
    {
        [NativeDisableParallelForRestriction] public NativeArray<float3> copyArray;
        [ReadOnly] public ComponentTypeHandle<Translation> translationHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int firstEntityInQueryIndex )
        {
            NativeArray<Translation> batchTranslation = batchInChunk.GetNativeArray( translationHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                copyArray[ firstEntityInQueryIndex + i ] = batchTranslation[ i ].Value;
            }
        }
    }
    [BurstCompile]
    private struct CopyVelocitesJob : IJobEntityBatchWithIndex
    {
        [NativeDisableParallelForRestriction] public NativeArray<float3> copyArray;
        [ReadOnly] public ComponentTypeHandle<Velocity> velocityHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int firstEntityInQueryIndex )
        {
            NativeArray<Velocity> batchVelocity = batchInChunk.GetNativeArray( velocityHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                copyArray[ firstEntityInQueryIndex + i ] = batchVelocity[ i ].Value;
            }
        }
    }
    [BurstCompile]
    private struct CopyMassJob : IJobEntityBatchWithIndex
    {
        [NativeDisableParallelForRestriction] public NativeArray<byte> copyArray;
        [ReadOnly] public ComponentTypeHandle<Mass> massHandle;

        public void Execute( ArchetypeChunk batchInChunk , int batchIndex , int firstEntityInQueryIndex )
        {
            NativeArray<Mass> batchMass = batchInChunk.GetNativeArray( massHandle );

            for ( int i = 0; i < batchInChunk.Count; i++ )
            {
                copyArray[ firstEntityInQueryIndex + i ] = ( byte ) batchMass[ i ].Value;
            }
        }
    }
    [BurstCompile]
    private struct FindCollidingPairsJob : IJobParallelFor
    {
        [ReadOnly] public float CELL_SIZE;
        [ReadOnly] public int CELL_CAPACITY;
        [ReadOnly] public int CELLS_ACROSS;
        [ReadOnly] public float RADIUS;
        [ReadOnly] public float AVOID_RADIUS;

        [ReadOnly] public NativeArray<ushort> grid;
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

            /*FixedList128<float> distances = new FixedList128<float>();
            FixedList128<int> otherUnitIndices = new FixedList128<int>();

            for ( int i = 0; i < newCells.Length; i++ )
            {
                int gridIndex = newCells[ i ] * CELL_CAPACITY;
                int count = 0;
                while ( grid[ gridIndex + count ] != VOID_CELL_VALUE && count < CELL_CAPACITY )
                {
                    int otherUnitIndex = grid[ gridIndex + count ];

                    if ( !collisionFlags.IsSet( otherUnitIndex ) )
                    {
                        otherUnitIndices.AddNoResize( otherUnitIndex );
                    }

                    count++;
                }
            }

            for ( int i = 0; i < otherUnitIndices.Length; i++ )
            {
                float px2 = copyPositions[ otherUnitIndices[ i ] ].x;
                float pz2 = copyPositions[ otherUnitIndices[ i ] ].z;
                float distance = math.sqrt( ( px - px2 ) * ( px - px2 ) + ( pz - pz2 ) * ( pz - pz2 ) );
                distances[ i ] = distance;
            }

            for ( int i = 0; i < distances.Length; i++ )
            {
                if ( distances[ i ] < AVOID_RADIUS )
                {
                    collidingPairs.Enqueue( new CollidingPair
                    {
                        unit1 = ( ushort ) index ,
                        unit2 = ( ushort ) otherUnitIndices[ i ] ,
                        distance = distances[ i ]
                    } );
                }
            }*/

            for ( int i = 0; i < newCells.Length; i++ )
            {
                int gridIndex = newCells[ i ] * CELL_CAPACITY;
                int count = 0;

                while ( grid[ gridIndex + count ] != VOID_CELL_VALUE && count < CELL_CAPACITY )
                {
                    int otherUnitIndex = grid[ gridIndex + count ];

                    if ( !collisionFlags.IsSet( otherUnitIndex ) )
                    {
                        float px2 = copyPositions[ otherUnitIndex ].x;
                        float pz2 = copyPositions[ otherUnitIndex ].z;
                        float distance = math.sqrt( ( px - px2 ) * ( px - px2 ) + ( pz - pz2 ) * ( pz - pz2 ) );

                        if ( distance < AVOID_RADIUS )
                        {
                            collidingPairs.Enqueue( new CollidingPair
                            {
                                unit1 = ( ushort ) index ,
                                unit2 = ( ushort ) otherUnitIndex ,
                                distance = distance
                            } );
                        }
                    }

                    count++;
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

    private struct UpdateCellData
    {
        public int unitID;
        public int oldCell;
        public int newCell;
    }
    private struct CollidingPair
    {
        public ushort unit1;
        public ushort unit2;
        public float distance;
    }
}

// for collision detection sort by unit group first
// then do the same thing for all units too far from formation

// while in combat, the frontline of unit can be changed by clicing on a different enemy unit

// for pushing through enemy units
// once move order is given the group will have countdown immunity for move order
// after like 1-3 seconds, will stop moving because still inside/within other group
// when fighting other group, units move forward incrementally
// during a charge, units are given a force of move plus charge
// as soon as unit hits a target, a countdown starts, and unit stops charging one counter is done
// can charge at specific location just charging through everything good for routing enemies

// so unit chargea, units bunch up but they stop their forward charge one by one as they hit enemies
// as they stand there all units always bubble away from each other to like half a meter apart
// units every second or 2 apply forward force to simulate push and pull
// in no formation this is basic combat
// in formation they do this but also flock to formation position
// formation position can change

// for formation fighting a formation has a shape, the shape is literally the rectangular (grid) of each unit in formation
// this means formation has sides (front, back, left, right) 
// so 4 frontlines
// while fighting, in a certain direction, units can move "forward"
// xxxxx   x 
// xxxxx  x xx
//        xxxx
// this happens when unit kills enemy units, or pushes it back
// when this happens, will check previous units until one if found, and it will move up to fill space
// this means when two formations are fighting, a temporary grid is setup there
// units cannot move into parts of grid that are occupied
// when two groups engage,
// a grid is created large enough to encompass both units shapes, and the "battlefield" for those two units is the grid
// each gridspot can hold one entity
// if an enemy takes allies grid spot, lets say through a push or through a charge,
// the ally is then "lost", it doesnt have a spot but still tries to push back to its old spot until it retakes it or dies or finds another spot
// if a unit is in defensive formation, it still has the grid but does not advance or otherwise purposely change shape
// if for example a formation gets charged and a bunch of frontliners are pushed back, will be detected and formation will move back

/*float4 AABB = new float4(
    px - RADIUS ,
    pz - RADIUS ,
    px + RADIUS ,
    pz + RADIUS );
float4 minMaxX = new float4(
    AABB.x ,
    AABB.z ,
    AABB.x ,
    AABB.z );
float4 minMaxZ = new float4(
    AABB.y ,
    AABB.w ,
    AABB.y ,
    AABB.w );
int4 neighbours = ( int4 ) ( math.floor( minMaxX / CELL_SIZE ) + math.floor( minMaxZ / CELL_SIZE ) * CELLS_ACROSS );          

FixedList128<int> newCellsSIMD = new FixedList128<int>();
newCellsSIMD.Add( neighbours.x );
if ( neighbours.x != neighbours.y )
    newCellsSIMD.Add( neighbours.y );
if ( neighbours.z != neighbours.x )
    newCellsSIMD.Add( neighbours.z );
if ( neighbours.y != neighbours.z )
    newCellsSIMD.Add( neighbours.w );

FixedList128<int> units = new FixedList128<int>();

for ( int i = 0; i < newCellsSIMD.Length; i++ )
{
    int gridIndex = newCellsSIMD[ i ] * CELL_CAPACITY;
    int count = 0;

    while ( grid[ gridIndex + count ] != VOID_CELL_VALUE && count < CELL_CAPACITY )
    {
        int otherUnitIndex = grid[ gridIndex + count ];

        if ( !collisionFlags.IsSet( otherUnitIndex ) )
            units.Add( otherUnitIndex );

        count++;
    }
}

FixedList128<bool> colliders = new FixedList128<bool>();
FixedList128<float> distances = new FixedList128<float>();

int si = 0;
for ( si = 0; si < units.Length - 4; si += 4 )
{
    float4 px2 = new float4(
        copyPositions[ units[ si ] ].x ,
        copyPositions[ units[ si + 1 ] ].x ,
        copyPositions[ units[ si + 2 ] ].x ,
        copyPositions[ units[ si + 3 ] ].x );
    float4 pz2 = new float4(
        copyPositions[ units[ si ] ].z ,
        copyPositions[ units[ si + 1 ] ].z ,
        copyPositions[ units[ si + 2 ] ].z ,
        copyPositions[ units[ si + 3 ] ].z );

    float4 distance = math.sqrt( ( px - px2 ) * ( px - px2 ) + ( pz - pz2 ) * ( pz - pz2 ) );
    bool4 collides = distance < RADIUS;

    distances.Add( distance.x );
    distances.Add( distance.y );
    distances.Add( distance.z );
    distances.Add( distance.w );

    colliders.Add( collides.x );
    colliders.Add( collides.y );
    colliders.Add( collides.z );
    colliders.Add( collides.w );
}
for ( int i = si; i < units.Length; i++)
{
    float px2 = copyPositions[ units[ i ] ].x;
    float pz2 = copyPositions[ units[ i ] ].z;
    float distance = math.sqrt( ( px - px2 ) * ( px - px2 ) + ( pz - pz2 ) * ( pz - pz2 ) );
    bool collides = distance < RADIUS;

    distances.Add( distance );
    colliders.Add( collides );
}

for ( int i = 0; i < colliders.Length; i++ )
{
    if ( colliders[ i ] )
    {
        collidingPairs.Enqueue( new CollidingPair
        {
            unit1 = ( ushort ) index ,
            unit2 = ( ushort ) units[ i ] ,
            distance = distances[ i ]
        } );
    }
}*/