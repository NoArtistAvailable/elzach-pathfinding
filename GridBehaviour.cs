using System.Diagnostics;
using System.Linq;
using elZach.Common;
using UnityEngine;
using UnityEngine.InputSystem;
using Debug = UnityEngine.Debug;

namespace elZach.PathFinding
{
    public class GridBehaviour : MonoBehaviour
    {
        public Vector3Int extents = new Vector3Int(8, 5, 8);
        public float minHeight = 2f;
        public LayerMask raycastMask = -1;
        public PathGrid grid;

        [SerializeField] private Button<GridBehaviour> getGridButton = new Button<GridBehaviour>(x => x.grid = new PathGrid(x.transform, x.extents, x.raycastMask));
        
        private PathGrid.PathNode firstNode;
        private int maximumStepHeight;
        
        private LineRenderer _line;
        LineRenderer line {get
        {
            if (!_line) _line = GetComponent<LineRenderer>();
            return _line;
        }}
        
        void Start()
        {
            grid = new PathGrid(transform, extents, raycastMask, minHeight);
        }

        void Update()
        {
            if (firstNode == null) return;
            var mouseNode = GetMouseNode();
            if (mouseNode == null) return;
            
            bool HeightCondition(PathGrid.PathNode current, PathGrid.PathNode neighbour)
            {
                return neighbour.Position.y - current.Position.y <= maximumStepHeight;
            }
            
            var path = grid.FindPath(firstNode, mouseNode, HeightCondition);
            if (path != null && path.Count > 1)
            {
                if (line)
                {
                    line.positionCount = path.Count + 1;
                    var pos = firstNode.Position + new Vector3(0.5f, 0.5f, 0.5f);
                    line.SetPosition(0, pos + ((Vector3) (path[0].Position - firstNode.Position)).normalized * 0.5f);
                    for(int i=0; i < path.Count; i++) line.SetPosition(i+1,path[i].Position + new Vector3(0.5f, 0.5f, 0.5f));
                }
                else for (int i = 1; i < path.Count; i++)
                {
                    Debug.DrawLine(
                        path[i - 1].Position + new Vector3(0.5f, 0.5f, 0.5f),
                        path[i].Position + new Vector3(0.5f, 0.5f, 0.5f),
                        Color.green);
                }
            }
        }

        public PathGrid.PathNode GetMouseNode()
        {
            if (!Physics.Raycast(Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue()), out var hit)) return null;
            return grid.WorldPosToNode(hit.point);
        }

        
        public void ActivatePath(Vector3 pos, int move, int maximumStepHeight)
        {
            Debug.DrawRay(pos, Vector3.up, Color.blue, 3f);
            var node = grid.WorldPosToNode(pos);
            this.maximumStepHeight = maximumStepHeight;
            if (node != null)
            {
                Debug.DrawRay(node.Position + new Vector3(0.5f, 0f, 0.5f), Vector3.up, Color.green, 3f);
                firstNode = node;

                bool HeightCondition(PathGrid.PathNode current, PathGrid.PathNode neighbour)
                {
                    return neighbour.Position.y - current.Position.y <= maximumStepHeight;
                }
                
                var reachables = grid.GetInRange(node, move, HeightCondition);
                var borderPoints =
                    grid.BorderFromNodes(reachables.Select<PathGrid.PathNode, PathGrid.Node>(x => x));
                for (var i = 1; i < borderPoints.Count; i++)
                    Debug.DrawLine(borderPoints[i - 1] + Vector3.one * 0.5f,
                        borderPoints[i] + Vector3.one * 0.5f,
                        Color.blue, 5f);
                Debug.DrawLine(borderPoints[^1] + Vector3.one * 0.5f, borderPoints[0] + Vector3.one * 0.5f,
                    Color.blue, 5f);
            }
        }

        void OnDrawGizmosSelected()
        {

            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireCube(Vector3.zero, extents * 2);
#if UNITY_EDITOR
            if (grid == null) return;
            UnityEditor.Handles.color = new Color(1f, 1f, 1f, 0.1f);
            foreach (var node in grid.Nodes)
                UnityEditor.Handles.DrawWireDisc(node.Position + new Vector3(0.5f, 0f, 0.5f), Vector3.up, 0.25f);
#endif
        }
    }
}
