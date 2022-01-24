using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class AStar
{

    public static Vector2Int[] ComputeShortestPath(float[,] traversability, Vector2Int startNode, Vector2Int goalNode)
    {

        Debug.Log(startNode);
        Debug.Log(goalNode);

        PriorityQueue<Vector2Int> discoveredNodes = new PriorityQueue<Vector2Int>();
        discoveredNodes.Insert(startNode, 0);

        Vector2Int[,] previousNodes = new Vector2Int[traversability.GetLength(0), traversability.GetLength(1)];

        float[,] pathCost = new float[traversability.GetLength(0), traversability.GetLength(1)];
        float[,] heurCost = new float[traversability.GetLength(0), traversability.GetLength(1)];

        for (int i = 0; i < pathCost.GetLength(0); i++)
        {
            for (int j = 0; j < pathCost.GetLength(1); j++)
            {
                pathCost[i, j] = float.PositiveInfinity;
                heurCost[i, j] = float.PositiveInfinity;
                previousNodes[i, j] = new Vector2Int(-1, -1);
            }
        }

        pathCost[startNode.x, startNode.y] = 0;
        heurCost[startNode.x, startNode.y] = Vector2Int.Distance(startNode, goalNode);

        while (discoveredNodes.Count() != 0)
        {
            Vector2Int currentNode = discoveredNodes.Pop();

            if (currentNode == goalNode)
            {
                // Reconstruct the path
                return reconstructPath(previousNodes, goalNode);
            }

            foreach (Vector2Int node in getNeighbors(currentNode, traversability))
            {
                float currentCost = pathCost[currentNode.x, currentNode.y] + 1; //Add 1 as it is the cost of a transition between two nodes
                if (currentCost < pathCost[node.x, node.y])
                {
                    previousNodes[node.x, node.y] = currentNode;
                    pathCost[node.x, node.y] = currentCost;
                    heurCost[node.x, node.y] = currentCost + Vector2Int.Distance(node, goalNode);

                    discoveredNodes.Insert(node, heurCost[node.x, node.y]);
                }
            }

        }

        return null;

    }


    private static Vector2Int[] getNeighbors(Vector2Int node, float[,] traversability)
    {
        //Neighbors in 4-connectivity
        List<Vector2Int> neighbors = new List<Vector2Int>();

        for (int i = 0; i < 8; i++)
        {
            if (i % 2 == 0) // Neighbors in 4-connexity
            {
                int j = i / 2;
                Vector2Int neighbor = j % 2 == 0 ? new Vector2Int(node.x + j - 1, node.y) : new Vector2Int(node.x, node.y + j - 2);

                if (isValidPosition(neighbor, traversability))
                {
                    neighbors.Add(neighbor);
                }
            }
            else // Neighbor in diagonal
            {
                // We check if the neighbors +1 and -1 are valid and add the diagonal neighbor if so (for the diagoal to be valid, the two non-diagonal paths must be clear)
                int jLeft = ((i - 1) % 8) / 2;
                Vector2Int neighborLeft = jLeft % 2 == 0 ? new Vector2Int(node.x + jLeft - 1, node.y) : new Vector2Int(node.x, node.y + jLeft - 2);
                int jRight = ((i + 1) % 8) / 2;
                Vector2Int neighborRight = jRight % 2 == 0 ? new Vector2Int(node.x + jRight - 1, node.y) : new Vector2Int(node.x, node.y + jRight - 2);

                Vector2Int neighbor = new Vector2Int(node.x + jLeft % 2 - 1, node.y + jLeft / 2 - 1);

                if (isValidPosition(neighborLeft, traversability) && isValidPosition(neighborRight, traversability) && isValidPosition(neighbor, traversability))
                {
                    neighbors.Add(neighbor);
                }
            }
        }

        return neighbors.ToArray();
    }

    private static bool isValidPosition(Vector2Int position, float[,] traversability)
    {
        if (position.x >= 0 && position.y >= 0 &&
            position.x < traversability.GetLength(0) && position.y < traversability.GetLength(1) &&
            traversability[position.x, position.y] == 0)
        {
            return true;
        }

        return false;
    }

    private static Vector2Int[] reconstructPath(Vector2Int[,] previousNodes, Vector2Int currentNode)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        path.Add(currentNode);

        while (previousNodes[currentNode.x, currentNode.y] != new Vector2Int(-1, -1))
        {
            currentNode = previousNodes[currentNode.x, currentNode.y];
            path.Add(currentNode);
        }
        path.Reverse();
        return path.ToArray();
    }
}
