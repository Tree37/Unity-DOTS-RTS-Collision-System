using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

public class MapGenerator : MonoBehaviour
{
    public GameObject map;

    public int mapSize;
    public int mapScale;
    public bool autoUpdate;

    public int seed;
    public bool useFlatShading;
    public float noiseScale;
    public int octaves;
    [Range( 0 , 1 )]
    public float persistance;
    public float lacunarity;
    public float meshHeightMultiplier;
    public AnimationCurve meshHeightCurve;
    public Vector2 offset;

    public BasicTerrainType[] regions;

    private void Start()
    {
        MeshData meshData = GenerateMap();
        World world = World.DefaultGameObjectInjectionWorld;
        SimulationSystemGroup group = world.GetOrCreateSystem<SimulationSystemGroup>();
        UnitMovementSystem movementSystem = world.GetOrCreateSystem<UnitMovementSystem>();
        movementSystem.SetVertices( meshData.collisionVertices , mapSize , mapScale );
        group.AddSystemToUpdateList( world.GetOrCreateSystem<UnitMovementSystem>() );
    }

    public MeshData GenerateMap()
    {
        float[,] noiseMap = Noise.GenerateNoiseMap( mapSize , seed , noiseScale , octaves , persistance , lacunarity , offset );

        Color[] colorMap = new Color[ mapSize * mapSize ];

        for ( int y = 0; y < mapSize; y++ )
        {
            for ( int x = 0; x < mapSize; x++ )
            {
                float currentHeight = noiseMap[ x , y ];
                for ( int i = 0; i < regions.Length; i++ )
                {
                    if ( currentHeight <= regions[ i ].height )
                    {
                        colorMap[ y * mapSize + x ] = regions[ i ].color;
                        break;
                    }
                }
            }
        }

        MeshData meshData = GenerateTerrainMesh( noiseMap , mapSize , meshHeightMultiplier , meshHeightCurve , useFlatShading );
        DrawMeshBasic( meshData , TextureFromColorMap( colorMap , mapSize ) );

        return meshData;
    }
    public MeshData GenerateTerrainMesh( float[,] heightMap , int mapSize , float heightMultiplier , AnimationCurve meshHeightCurve , bool useFlatShading )
    {
        MeshData meshData = new MeshData( mapSize , useFlatShading );
        int vertexIndex = 0;

        for ( int y = 0; y < mapSize; y++ )
        {
            for ( int x = 0; x < mapSize; x++ )
            {
                float height = meshHeightCurve.Evaluate( heightMap[ x , y ] ) * heightMultiplier * mapScale;
                meshData.vertices[ vertexIndex ] = new Vector3( x * mapScale , height , y * mapScale );
                meshData.collisionVertices[ vertexIndex ] = new Vector3( x * mapScale , height , y * mapScale );
                meshData.uvs[ vertexIndex ] = new Vector2( x / ( float ) mapSize , y / ( float ) mapSize );

                if ( x < mapSize - 1 && y < mapSize - 1 )
                {
                    meshData.AddTriangle( vertexIndex , vertexIndex + mapSize + 1 , vertexIndex + mapSize );
                    meshData.AddTriangle( vertexIndex + mapSize + 1 , vertexIndex , vertexIndex + 1 );
                }

                vertexIndex++;
            }
        }

        meshData.ProcessMesh();

        return meshData;
    }
    public Texture2D TextureFromColorMap( Color[] colorMap , int mapSize )
    {
        Texture2D texture = new Texture2D( mapSize , mapSize );
        texture.filterMode = FilterMode.Point;
        texture.wrapMode = TextureWrapMode.Clamp;
        texture.SetPixels( colorMap );
        texture.Apply();
        return texture;
    }
    public void DrawMeshBasic( MeshData meshData , Texture2D texture )
    {
        map.GetComponent<MeshFilter>().sharedMesh = meshData.CreateMesh();
        map.GetComponent<MeshRenderer>().sharedMaterial.mainTexture = texture;
    }
}

[System.Serializable]
public struct BasicTerrainType
{
    public string name;
    public float height;
    public Color color;
}

public class MeshData
{
    public Vector3[] vertices;
    public Vector3[] collisionVertices;
    public int[] triangles;
    public Vector2[] uvs;

    private int triangleIndex;
    private bool useFlatShading;

    public MeshData( int mapSize , bool _useFlatShading )
    {
        vertices = new Vector3[ mapSize * mapSize ];
        collisionVertices = new Vector3[ mapSize * mapSize ];
        triangles = new int[ ( mapSize - 1 ) * ( mapSize - 1 ) * 6 ];
        uvs = new Vector2[ mapSize * mapSize ];

        triangleIndex = 0;
        useFlatShading = _useFlatShading;
    }

    public void AddTriangle( int a , int b , int c )
    {
        triangles[ triangleIndex + 0 ] = c;
        triangles[ triangleIndex + 1 ] = b;
        triangles[ triangleIndex + 2 ] = a;

        triangleIndex += 3;
    }

    public void ProcessMesh()
    {
        if ( useFlatShading )
            FlatShading();
    }

    public Mesh CreateMesh()
    {
        Mesh mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uvs;
        //mesh.normals = CalculateNormals();
        mesh.RecalculateNormals();

        return mesh;
    }

    private Vector3[] CalculateNormals()
    {
        Vector3[] vertexNormals = new Vector3[ vertices.Length ];
        int triangleCount = triangles.Length / 3;

        for ( int i = 0; i < triangleCount; i++ )
        {
            int normalTriangleIndex = i * 3;

            int vertIndexA = triangles[ normalTriangleIndex + 0 ];
            int vertIndexB = triangles[ normalTriangleIndex + 1 ];
            int vertIndexC = triangles[ normalTriangleIndex + 2 ];

            Vector3 triangleNormal = SurfaceNormalFromIndices( vertIndexA , vertIndexB , vertIndexC );

            vertexNormals[ vertIndexA ] += triangleNormal;
            vertexNormals[ vertIndexB ] += triangleNormal;
            vertexNormals[ vertIndexC ] += triangleNormal;
        }

        for ( int i = 0; i < vertexNormals.Length; i++ )
            vertexNormals[ i ].Normalize();

        return vertexNormals;
    }

    private Vector3 SurfaceNormalFromIndices( int indexA , int indexB , int indexC )
    {
        Vector3 pointA = vertices[ indexA ];
        Vector3 pointB = vertices[ indexB ];
        Vector3 pointC = vertices[ indexC ];

        Vector3 sideAB = pointB - pointA;
        Vector3 sideAC = pointC - pointA;

        return Vector3.Cross( sideAB , sideAC ).normalized;
    }

    private void FlatShading()
    {
        Vector3[] flatShadedVertices = new Vector3[ triangles.Length ];
        Vector2[] flatShadedUvs = new Vector2[ triangles.Length ];

        for ( int i = 0; i < triangles.Length; i++ )
        {
            flatShadedVertices[ i ] = vertices[ triangles[ i ] ];
            flatShadedUvs[ i ] = uvs[ triangles[ i ] ];
            triangles[ i ] = i;
        }

        vertices = flatShadedVertices;
        uvs = flatShadedUvs;
    }
}
