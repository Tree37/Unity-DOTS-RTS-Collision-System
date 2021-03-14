using UnityEngine;
using Unity.Entities;

public class GameManager : MonoBehaviour
{
    public Mesh unitMesh;
    public Material unitMaterial;

    public UnitSpawner unitSpawner;

    private void Start()
    {
        unitSpawner = new UnitSpawner( unitMesh , unitMaterial );
    }

    private void DrawGrid()
    {
        for ( int i = 0; i < 100; i++ )
        {
            for ( int j = 0; j < 100; j++ )
            {
                Vector3 start = new Vector3( j , 0.1f , i );
                Vector3 end = new Vector3( j , 0.1f , i + 100 );

                Debug.DrawLine( start , end , Color.blue );
            }
        }
        for ( int i = 0; i < 100; i++ )
        {
            for ( int j = 0; j < 100; j++ )
            {
                Vector3 start = new Vector3( i , 0.1f , j );
                Vector3 end = new Vector3( i + 100 , 0.1f , j );

                Debug.DrawLine( start , end , Color.blue );
            }
        }
    }

    private void RunUnits()
    {

    }
}
