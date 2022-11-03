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

        public List<PathNode> FindPath(PathNode from, PathNode to, Func<PathNode, PathNode, bool> pathCondition = null,
            Func<PathNode, bool> exclusionCondition = null) =>
            PathFinding<PathNode>.FindPath(Nodes, from, to, pathCondition, exclusionCondition);

        public List<PathNode> GetInRange(PathNode startNode, int distance,
            Func<PathNode, PathNode, bool> pathCondition = null, Func<PathNode, bool> exclusionCondition = null) =>
            PathFinding<PathNode>.GetInRange(Nodes, startNode, distance, pathCondition, exclusionCondition);

        public List<PathNode> GetInRange(Vector3 position, int distance,
            Func<PathNode, PathNode, bool> pathCondition = null, Func<PathNode, bool> exclusionCondition = null) =>
            GetInRange(WorldPosToNode(position), distance, pathCondition, exclusionCondition);

        public PathNode WorldPosToNode(Vector3 position)
        {
            var pos = new Vector3Int(
                Mathf.FloorToInt(position.x),
                Mathf.RoundToInt(position.y),
                Mathf.FloorToInt(position.z));
            return Nodes.FirstOrDefault(x => ((Node)x).Position == pos);
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
                        if (!Physics.Raycast(hit.point, Vector3.up, minHeight, raycastMask))
                        {
                            var y = Mathf.RoundToInt(hit.point.y);
                            var node = new Node(new Vector3Int(x, y, z), x + extents.x, z + extents.z,
                                nodeGrid);
                            Nodes.Add(node);
                            AddToGrid(x + extents.x, z + extents.z, node);
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
        public class Node : PathNode
        {
            public Vector3Int Position;
            // TODO: figure out if we can attach pathfinding values to any kind of node instead of inheriting
            // conditions for pathfinding should only be that we've got a Position and neighbours
            // can we somehow convert the PathNode class into an IPathNode interface - would that make sense?
            // IPath Node would need to implement Distance(), DistanceCost() and Neighbours
            // then we could maybe store fCost/gCost/hCost/cameFrom in a lookUpTable while a PathFinding function is running?
            // if we store neighbours as indices of a lookup we could also structify PathNode and fit it into Unity JobSystem

            public Node(Vector3Int position, int gridX, int gridY, List<PathNode>[,] grid)
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

            private const int CostStraight = 10;
            private const int CostDiagonal = 14;

            public override int DistanceCost(PathNode node) => DistanceCost((Node)node);

            private int DistanceCost(Node node)
            {
                int xDistance = Mathf.Abs(Position.x - node.Position.x);
                int zDistance = Mathf.Abs(Position.z - node.Position.z);
                int remaining = Mathf.Abs(xDistance - zDistance);
                return CostDiagonal * Mathf.Min(xDistance, zDistance) + CostStraight * remaining;
            }

            public override int Distance(PathNode node) => Distance((Node)node);

            private int Distance(Node node) =>
                Mathf.Abs(Position.x - node.Position.x) + Mathf.Abs(Position.z - node.Position.z);

        }

        //[Serializable]

    }

    public abstract class PathNode
    {
        public List<PathNode> m_Neighbours = new List<PathNode>();

        // these values only have to exists for any given path finding
        internal int fCost;
        internal int gCost;
        internal int hCost;
        internal PathNode cameFrom;

        internal int CalculateFCost() => gCost + hCost;

        public abstract int DistanceCost(PathNode node);
        public abstract int Distance(PathNode node);

        
    }

    public static class PathFinding<T> where T : PathNode
    {
        internal static List<T> TraceBack(T origin)
        {
            List<T> path = new List<T>();
            var current = origin;
            while (current.cameFrom != null)
            {
                path.Add(current);
                current = (T) current.cameFrom;
            }

            path.Reverse();
            return path;
        }
        
        public static List<T> FindPath(List<T> allNodes, T from, T to,
            Func<T, T, bool> pathCondition = null, Func<T, bool> exclusionCondition = null)
        {
            // var targetPosition = to.Position;
            var openList = new List<T>() { from };
            var closedList = new List<T>();
            pathCondition ??= (current, neighbour) => true;
            exclusionCondition ??= (current) => false;

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
                openList.Remove(current);
                closedList.Add(current);
                if (current == to)
                {
                    //finished
                    return TraceBack(to);
                }

                foreach (var neigh in current.m_Neighbours)
                {
                    //var neighbour = allNodes[neighbourIndex];
                    var neighbour = (T)neigh;
                    if (closedList.Contains(neighbour)) continue;
                    if (exclusionCondition(neighbour))
                    {
                        closedList.Add(neighbour);
                        continue;
                    }

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
        
        public static List<T> GetInRange(IEnumerable<T> allNodes, T startNode, int distance,
            Func<T, T, bool> pathCondition = null, Func<T, bool> exclusionCondition = null)
        {
            foreach (var node in allNodes) node.gCost = int.MaxValue;

            var openList = new List<T>() { startNode };
            var closedList = new List<T>();
            var inRange = new List<T>() { startNode };
            pathCondition ??= (current, neighbour) => true;
            exclusionCondition ??= (node) => false;

            startNode.gCost = 0;

            while (openList.Count > 0)
            {
                T current = openList[0];
                foreach (T neighbour in current.m_Neighbours)
                {
                    //var neighbour = allNodes[neighbourIndex];
                    if (closedList.Contains(neighbour)) continue;
                    if (exclusionCondition.Invoke(neighbour))
                    {
                        closedList.Add(neighbour);
                        continue;
                    }

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
