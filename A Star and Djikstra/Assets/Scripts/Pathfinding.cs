using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;

public class Pathfinding : MonoBehaviour {

	public Transform seeker, target;
    public bool dijkstra;
	
	Grid grid;

	void Awake() {
		grid = GetComponent<Grid>();
	}

	void Update() {
		FindPath(seeker.position,target.position);
	}

	void FindPath(Vector3 startPos, Vector3 targetPos) {

        Stopwatch sw = new Stopwatch();
        sw.Start();
        Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
        print("grid max size: " + grid.MaxSize);
        HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);
        int node_counter = 0;

		while (openSet.Count > 0) {
			Node currentNode = openSet.RemoveFirst();
			closedSet.Add(currentNode);
            node_counter = node_counter + 1;

			if (currentNode == targetNode) {
                sw.Stop();
                print("Path found: " + sw.ElapsedMilliseconds + " ms");
                RetracePath(startNode,targetNode);
                print("Closed set size" + closedSet.Count);
                print("Total nodes: " + node_counter);
                print("openset size: " + openSet.Count);
                return;
			}

			foreach (Node neighbour in grid.GetNeighbours(currentNode)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
				int newMovementCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
				if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newMovementCostToNeighbour;
                    if (dijkstra)
                    {
                        neighbour.hCost = 0;
                    }
                    else
                    {
                        neighbour.hCost = GetDistance(neighbour, targetNode);
                    }
                    neighbour.parent = currentNode;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
					else {
						openSet.UpdateItem(neighbour);
					}
				}
			}
		}
	}

	void RetracePath(Node startNode, Node endNode) {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
            
		}
		path.Reverse();

		grid.path = path;
       

	}

	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}


}
