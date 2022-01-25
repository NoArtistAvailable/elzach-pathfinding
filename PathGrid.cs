using System;
using System.Collections.Generic;
using System.Linq;
using elZach.Common;
using UnityEngine;

namespace elZach.PathFinding
{
    public class PathGrid
    {
        public readonly List<PathNode> Nodes;
        public List<PathNode> FindPath(PathNode from, PathNode to, Func<PathNode, PathNode, bool> pathCondition = null) =>
            PathNode.FindPath(Nodes, from, to, pathCondition);

        public List<PathNode> GetInRange(PathNode startNode, int distance, Func<PathNode, PathNode, bool> pathCondition = null) =>
            PathNode.GetInRange(Nodes, startNode, distance, pathCondition);
        
        public List<PathNode> GetInRange(Vector3 position, int distance, Func<PathNode, PathNode, bool> pathCondition = null) =>
            GetInRange(WorldPosToNode(position), distance, pathCondition);
        
        public PathNode WorldPosToNode(Vector3 position)
        {
            var pos = new Vector3Int(Mathf.FloorToInt(position.x), Mathf.FloorToInt(position.y),
                Mathf.FloorToInt(position.z));
            return Nodes.FirstOrDefault(x => x.Position == pos);
        }

        public PathGrid(Transform transform, Vector3Int extents, LayerMask raycastMask, float minHeight = 0f)
        {
            Nodes = new List<PathNode>();
            var nodeGrid = new List<PathNode>[extents.x * 2, extents.z * 2];
            if (minHeight == 0f) minHeight = extents.y * 2f;
            void AddToGrid(int x, int z, PathNode node)
            {
                nodeGrid[x, z] ??= new List<PathNode>();
                nodeGrid[x, z].Add(node);
            }
            for (int x = -extents.x; x < extents.x; x++)
            for (int z = -extents.z; z < extents.z; z++)
            {
                var rayPos = new Vector3(x + 0.5f + transform.position.x, transform.position.y + extents.y,
                    z + 0.5f + transform.position.z);
                RaycastHit hit;
                while (rayPos.y > transform.position.y - extents.y)
                    if (Physics.Raycast(rayPos, Vector3.down, out hit, extents.y * 2f, raycastMask))
                    {
                        if (hit.collider.CompareTag("Walkable"))
                        {
                            if (!Physics.Raycast(hit.point, Vector3.up, minHeight, raycastMask))
                            {
                                var y = Mathf.RoundToInt(hit.point.y);
                                var node = new PathNode(new Vector3Int(x, y, z), x + extents.x, z + extents.z,
                                    nodeGrid);
                                Nodes.Add(node);
                                AddToGrid(x + extents.x, z + extents.z, node);
                            }
                        }
                        rayPos = hit.point - Vector3.up * minHeight;
                    }
                    else break;
            }
        }

        public List<Vector3Int> BorderFromNodes(IEnumerable<Node> nodes)
        {
            var mins = new List<Vector3Int>();
            var maxs = new List<Vector3Int>();
            //get top & bottom of one dimension eg x
            var minX = nodes.Min(x => x.Position.x);
            var maxX = nodes.Max(x => x.Position.x);
            //then iterate over every point in between and get top & bottom of that respective point in the other dimension eg maxY & minY for every x
            for (int i = minX; i < maxX; i++)
            {
                int x = i;
                var range = nodes.Where(node => node.Position.x == x);
                var minZ = range.MinBy(node => node.Position.z);
                var maxZ = range.MaxBy(node => node.Position.z);
                mins.Add(minZ.Position);
                maxs.Insert(0, maxZ.Position);
            }

            return mins.Concat(maxs).ToList();
        }

        //[Serializable]
        public class Node
        {
            public Vector3Int Position;
            // TODO: figure out if we can attach pathfinding values to any kind of node instead of inheriting
            // conditions for pathfinding should only be that we've got a Position and neighbours
            // can we somehow convert the PathNode class into an IPathNode interface - would that make sense?
            // IPath Node would need to implement Position and Neighbours
            // then we could maybe store fCost/gCost/hCost/cameFrom in a lookUpTable while a PathFinding function is running?
            // if we store neighbours as indices of a lookup we could also structify PathNode and fit it into Unity JobSystem
        }

        //[Serializable]
        public class PathNode : Node
        {
            public PathNode(Vector3Int position, int gridX, int gridY, List<PathNode>[,] grid)
            {
                Position = position;
                foreach (var potentialNodes in grid.GetNeighbours(gridX, gridY).Where(x => x != null))
                {
                    foreach (var pathNode in potentialNodes)
                    {
                        m_Neighbours.Add(pathNode);
                        pathNode.m_Neighbours.Add(this);
                    }
                }
            }

            private readonly List<PathNode> m_Neighbours = new List<PathNode>();

            private const int CostStraight = 10;
            private const int CostDiagonal = 14;

            // these values only have to exists for any given path finding
            private int fCost;
            private int gCost;
            private int hCost;
            private PathNode cameFrom;

            private int CalculateFCost() => gCost + hCost;

            private int DistanceCost(PathNode node)
            {
                int xDistance = Mathf.Abs(Position.x - node.Position.x);
                int zDistance = Mathf.Abs(Position.z - node.Position.z);
                int remaining = Mathf.Abs(xDistance - zDistance);
                return CostDiagonal * Mathf.Min(xDistance, zDistance) + CostStraight * remaining;
            }

            private int Distance(PathNode node) =>
                Mathf.Abs(Position.x - node.Position.x) + Mathf.Abs(Position.z - node.Position.z);

            private List<PathNode> TraceBack()
            {
                List<PathNode> path = new List<PathNode>();
                var current = this;
                while (current.cameFrom != null)
                {
                    path.Add(current);
                    current = current.cameFrom;
                }

                path.Reverse();
                return path;
            }

            internal static List<PathNode> FindPath(List<PathNode> allNodes, PathNode from, PathNode to,
                Func<PathNode, PathNode, bool> pathCondition)
            {
                // var targetPosition = to.Position;
                var openList = new List<PathNode>() {from};
                var closedList = new List<PathNode>();
                pathCondition ??= (current, neighbour) => true;

                foreach (var n in allNodes)
                {
                    n.gCost = int.MaxValue;
                    n.cameFrom = null;
                }

                var node = from;
                //
                node.gCost = 0;
                node.hCost = node.DistanceCost(to);
                node.fCost = node.CalculateFCost();

                while (openList.Count > 0)
                {
                    var current = openList.MinBy(x => x.fCost);
                    if (current == to)
                    {
                        //finished
                        return to.TraceBack();
                    }

                    openList.Remove(current);
                    closedList.Add(current);

                    foreach (var neighbour in current.m_Neighbours)
                    {
                        //var neighbour = allNodes[neighbourIndex];
                        if (closedList.Contains(neighbour)) continue;
                        if (!pathCondition.Invoke(current, neighbour)) continue;
                        int tentativeGCost = current.gCost + current.DistanceCost(neighbour);
                        if (tentativeGCost >= neighbour.gCost) continue;
                        neighbour.cameFrom = current;
                        neighbour.gCost = tentativeGCost;
                        neighbour.hCost = neighbour.DistanceCost(to);
                        neighbour.fCost = neighbour.CalculateFCost();
                        if (!openList.Contains(neighbour)) openList.Add(neighbour);
                    }
                }

                //found no paths in neighbours
                return null;
            }

            internal static List<PathNode> GetInRange(IEnumerable<PathNode> allNodes, PathNode startNode, int distance,
                Func<PathNode, PathNode, bool> pathCondition)
            {
                foreach (var node in allNodes) node.gCost = int.MaxValue;

                var openList = new List<PathNode>() {startNode};
                var closedList = new List<PathNode>();
                var inRange = new List<PathNode>() {startNode};
                pathCondition ??= (current, neighbour) => true;

                startNode.gCost = 0;

                while (openList.Count > 0)
                {
                    var current = openList[0];
                    foreach (var neighbour in current.m_Neighbours)
                    {
                        //var neighbour = allNodes[neighbourIndex];
                        if (closedList.Contains(neighbour)) continue;
                        if (!pathCondition.Invoke(current, neighbour)) continue;
                        int tentativeGCost = current.gCost + neighbour.Distance(current);
                        if (tentativeGCost > distance || tentativeGCost >= neighbour.gCost) continue;
                        neighbour.gCost = tentativeGCost;
                        if (!inRange.Contains(neighbour)) inRange.Add(neighbour);
                        if (!openList.Contains(neighbour)) openList.Add(neighbour);
                    }

                    openList.Remove(current);
                    closedList.Add(current);
                }

                return inRange;
            }

        }
    }
}
