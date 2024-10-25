import java.util.*;
/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node{
  int id;    //ID of each polygon node
  ArrayList<Wall> polygon;  //Walls that form a polygon
  ArrayList<Node> neighbors;  //Neighboring nodes (adjacent polygons)
  ArrayList<Wall> connections;  //Connections (shared walls) with neighbors
  HashMap<Node, Wall> neighborMap; //Mapping of neighbors to their connecting walls.
  PVector center;  //center of a polygon
   
  Node(int id, ArrayList<Wall> walls){
    this.id = id;  
    this.polygon = walls;
    this.connections = new ArrayList<Wall>(); 
    this.neighbors = new ArrayList<Node>();
    this.neighborMap = new HashMap<Node, Wall>();
    
    float xSum = 0;
    float ySum = 0;
    
    for(Wall w : polygon){
      xSum += w.start.x;
      ySum += w.start.y;
    }
    
    this.center = new PVector(xSum / polygon.size(), ySum / polygon.size());
  }
  
  //Adds a neighboring polygon and its connecting wall
  void addNeighborWall(Node neighbor, Wall wall){
    neighborMap.put(neighbor, wall);
  }
  //Adds a connection (shared wall) to the node
  void addConnection(Wall wall){
    connections.add(wall);
  }
  //Adds a neighboring polygon to the node
  void addNeighbor(Node neighbor){
    neighbors.add(neighbor);
  }
  
}

class NavMesh{   
  public ArrayList<Wall> wallList;          
  public ArrayList<Node> polygonList;    
  HashMap<Integer, Node> wallMap = new HashMap<Integer, Node>();  
  public ArrayList<PVector> reflexPoint; 
 
   
  void bake(Map map){
    //initializing
    ArrayList<Wall> walls = map.walls;
    wallList = new ArrayList<Wall>();         
    polygonList = new ArrayList<Node>();
    wallMap = new HashMap<Integer, Node>(); 
    reflexPoint = new ArrayList<PVector>();

    split(walls);

    for (Node polygon : polygonList) {
      if (polygon.connections.size() != polygon.neighbors.size()) {
        for (Wall connection : polygon.connections) {
          int connectionID = connection.getID();
          if (wallMap.containsKey(connectionID)) {
            Node neighbor = wallMap.get(connectionID);
            polygon.addNeighbor(neighbor);
            polygon.addNeighborWall(neighbor, connection);
          }
        }
      }
    }       
    printMap();
    printPolygonInfo();
  }
   
  //Recursively split all polygons in the list until there is only convex polygons left.
  void split(ArrayList<Wall> polygons) {
    int reflexPolygonIndex = isReflex(polygons);
    
    // If the polygon is already convex, add it to the convex polygon list
    if (reflexPolygonIndex == -1) {
      Node polygon = new Node(polygonList.size(), polygons);

      for (Wall wall : polygons) {
        if (wall.getID() != 0) {
          polygon.addConnection(wall);
          int wallID = wall.getID();         
          if (wallMap.containsKey(wallID)) {
            Node neighborNode = wallMap.get(wallID);
            polygon.addNeighbor(neighborNode);
            polygon.addNeighborWall(neighborNode, wall);
            wallMap.put(-wallID, polygon);
          }   
          else{
           wallMap.put(-wallID, polygon);
         }
       }
     }
     polygonList.add(polygon);
     return;
   } 
    
    // Polygon is concave, find the reflex point and fix point to connect
    PVector reflexPoint = polygons.get(reflexPolygonIndex).start;
    int fixPointIndex = findFixPoint(polygons, reflexPolygonIndex, reflexPoint);
    PVector fixPoint = polygons.get(fixPointIndex).start;

    // Ensure walls are ordered correctly for both polygons
    if (fixPointIndex < reflexPolygonIndex){
      int temp = fixPointIndex;
      fixPointIndex = reflexPolygonIndex;
      reflexPolygonIndex = temp;
      reflexPoint = polygons.get(reflexPolygonIndex).start;
      fixPoint = polygons.get(fixPointIndex).start;
    }

    // Create walls for splitting
    Wall normalWall = new Wall(reflexPoint, fixPoint);
    normalWall.setID(wallList.size() + 1);
    Wall reverseWall = new Wall(fixPoint, reflexPoint);
    reverseWall.setID(-wallList.size() - 1);
    
    // Add the new wall to the list
    wallList.add(normalWall);   

    // split walls into two new polygons
    ArrayList<Wall> frontPolygon = new ArrayList<>();
    ArrayList<Wall> backPolygon = new ArrayList<>();
    backPolygon.add(reverseWall);

    for (int i = 0; i < polygons.size(); i++) {
      if (i >= reflexPolygonIndex && i < fixPointIndex) {
        backPolygon.add(polygons.get(i));
      } 
      else{
        frontPolygon.add(polygons.get(i));
      }
    }
    
    frontPolygon.add(reflexPolygonIndex, normalWall);

    // Recursively split the new polygons
    split(frontPolygon);
    split(backPolygon);
  }

  int findFixPoint(ArrayList<Wall> polygons, int reflexPolygonIndex, PVector reflexPoint) {
    int fixPointIndex = 0;
    int startPoint = 2;
    boolean reachable = false;
    boolean reflexive = true;
  
    // Iterate through all points to find a valid fix point
    while (!reachable || !reflexive) {
      fixPointIndex = (reflexPolygonIndex + startPoint) % polygons.size();
      PVector fixPoint = polygons.get(fixPointIndex).start;
      Wall tempWall = new Wall(reflexPoint, fixPoint);
          
      reachable = canPlace(polygons, tempWall);
      int previousIndex = (reflexPolygonIndex - 1 + polygons.size()) % polygons.size();
  
      // Check if the new wall makes a convex polygon
      reflexive = polygons.get(previousIndex).normal.dot(tempWall.direction) < 0;
      startPoint++;
    }
    return fixPointIndex;
  }


  int isReflex(ArrayList<Wall> mapList){
    // Loop through walls to find an index with a reflex angle
    for (int i = 0; i < mapList.size(); i++) {
        int nextIndex = (i + 1) % mapList.size();
        Wall currentWall = mapList.get(i);
        Wall nextWall = mapList.get(nextIndex);

        // Check if the angle between current and next walls is reflex
        if (currentWall.normal.dot(nextWall.direction) >= 0) {
            return nextIndex;
        }
    }
    return -1; // No reflex angle found
}

   
  //Making sure the walls won't collide
  boolean canPlace (ArrayList<Wall> polygons, Wall tempWall){
    boolean valid = false;
    PVector from = PVector.add(tempWall.start, PVector.mult(tempWall.direction, 0.01));
    PVector to = PVector.add(tempWall.end, PVector.mult(tempWall.direction, -0.01));
    for(int n = 0; n<polygons.size(); n++){
      if(polygons.get(n).crosses(from, to)){
        return valid;
      }
    }
    valid = true;
    return valid;
  }
  
  //Printing into console
  void printWallCoords(ArrayList<Wall> Walls){
    for(int x = 0; x<Walls.size(); x++){
      print("\nWall " + x + " starts at point " + Walls.get(x).start + " and ends at point " + Walls.get(x).end );
    }
  }
  
  void printMap(){
    print("\nThe Map is:" + wallMap);
  }
  
  void printPolygonInfo() {
    for (Node polygon : polygonList) {
      print("\nPolygon ID: " + polygon.id);
      print("\nConnections:");
      for (Wall connection : polygon.connections) {
        print("\tID: " + connection.ID);
      }
      print("\n\tNeighbors:");
      for (Node neighbor : polygon.neighbors) {
        print("\tID: " + neighbor.id + ", Point: " + neighbor.center);
      }
    }
  }

   
  // Class that holds information for each node in the frontier
  class frontierNode {
    float heuristic;
    float pathLength;
    float aValue;
    Node currentNode;
    frontierNode prevNode;
  
    frontierNode(Node currentNode, frontierNode prevNode, float pathLength, PVector target) {
      this.currentNode = currentNode;
      this.prevNode = prevNode;
      this.heuristic = getPVectorDistance(currentNode.center, target);
      this.pathLength = pathLength;
      this.aValue = pathLength + heuristic;
    }
  }
  
  //Pathfinding function for boid
  ArrayList<PVector> findPath(PVector boidLocation, PVector desiredLocation) {
    ArrayList<PVector> pathList = new ArrayList<>();
  
    Node startNode = findContainingNode(boidLocation);
    Node endNode = findContainingNode(desiredLocation);
  
    // Ensure both start and end nodes are valid and different
    if (startNode != null && endNode != null && !startNode.equals(endNode)) {
      // Find the path of nodes between start and end nodes
      ArrayList<Node> nodePath = findNodePath(startNode, endNode);
  
      // Add waypoints (centers of connection walls) to the path list
      for (int i = 0; i < nodePath.size() - 1; i++) {
        Node currentNode = nodePath.get(i);
        Node nextNode = nodePath.get(i + 1);
        Wall connectingWall = currentNode.neighborMap.get(nextNode);
        pathList.add(connectingWall.center());
      }
    }
    pathList.add(desiredLocation);
    println("Amount of polygons passed: " + pathList.size());
    return pathList;
 }
  
  // Helper function to find which node contains the given point
  Node findContainingNode(PVector point) {
    for (Node node : polygonList) {
      if (pointContained(point, node.polygon)) {
        return node;
      }
    }
    return null;
  }
  
  // Function to check if a point is inside a polygon
  boolean pointContained(PVector given, ArrayList<Wall> polygon) {
    float crosses = 0;
    for (Wall wall : polygon) {
      if (wall.crosses(given, new PVector(given.x + 2 * width, given.y))) {
        crosses++;
      }
    }
    return (crosses % 2) != 0;
  }
  
  // Function to find the optimal node path using A*
  ArrayList<Node> findNodePath(Node boid, Node desired) {
    ArrayList<frontierNode> frontierNodes = new ArrayList<>();
    ArrayList<Node> expandedNodes = new ArrayList<>();
  
    frontierNodes.add(new frontierNode(boid, null, 0, desired.center));
  
    // Expand the frontier until the end is reached
    while (frontierNodes.get(0).currentNode != desired && !frontierNodes.isEmpty()) {
      frontierNode current = frontierNodes.remove(0);
      Node currentNode = current.currentNode;
  
      println("Expanding node at: " + currentNode.center);
  
      for (Node neighbor : currentNode.neighbors) {
        float newPathLength = current.pathLength + getPVectorDistance(currentNode.center, neighbor.center);
        frontierNodes.add(new frontierNode(neighbor, current, newPathLength, desired.center));
      }
  
      expandedNodes.add(currentNode);
      frontierNodes.sort(new frontierNodeComparator());
  
      // Remove already expanded nodes
      while (!frontierNodes.isEmpty() && expandedNodes.contains(frontierNodes.get(0).currentNode)) {
        frontierNodes.remove(0);
      }
    }
  
    // Trace back from the end to the start
    ArrayList<Node> finalPath = new ArrayList<>();
    if (!frontierNodes.isEmpty()) {
      frontierNode current = frontierNodes.get(0);
      finalPath.add(current.currentNode);
  
      while (current.prevNode != null) {
        current = current.prevNode;
        finalPath.add(0, current.currentNode);
      }
    }
    return finalPath;
  }
  
  // Comparator for frontier nodes based on A* value
  class frontierNodeComparator implements Comparator<frontierNode> {
    public int compare(frontierNode a, frontierNode b) {
      return Float.compare(a.aValue, b.aValue);
    }
  }
 
   
  void update(float dt)
  {
     draw();     
  }
   
  void draw()
  {
    /// use this to draw the nav mesh graph
    for (PVector point : reflexPoint) {
        stroke(0, 255, 0);
        circle(point.x, point.y, 50);
    }

    // Draw convex polygons and connections
    if (polygonList != null) {
      for (Node polygonNode : polygonList) {
        stroke(255, 0, 0);
        circle(polygonNode.center.x, polygonNode.center.y, 5);

        // Draw connections between neighbors
        for (Wall connection : polygonNode.connections) {
          line(connection.start.x, connection.start.y, connection.end.x, connection.end.y);
        }
      }
    } 
  }
}
