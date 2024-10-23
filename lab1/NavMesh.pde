import java.util.Comparator;
import java.util.HashMap;
import java.util.Random;
import java.util.*;
/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node{
  int id;    //each polygon is given a unique ID
  ArrayList<Wall> polygon;    
  PVector center;   
  ArrayList<Node> neighbors;    
  HashMap<Node, Wall> neighborToWall;    //the map to quickly find a wall based on which neighbor is being analyzed
  ArrayList<Wall> connections;    //a seperate list of walls to make the design easier
   
  Node(int id, ArrayList<Wall> walls){
    this.id = id;  
    polygon = walls;
    center = calculateCenter(walls);
    neighbors = new ArrayList<Node>();
    neighborToWall = new HashMap<Node, Wall>();
    connections = new ArrayList<Wall>();   
  }
  
  //Calcuate the center of the polygon
  PVector calculateCenter (ArrayList<Wall> polygon){
    float xSum = 0;
    float ySum = 0;
    
    for(Wall w : polygon){
      xSum += w.start.x;
      ySum += w.start.y;
    }
    
    return new PVector(xSum / polygon.size(), ySum / polygon.size());
  }
  
  void addNeighbor(Node newNeighbor){
    neighbors.add(newNeighbor);
  }
  
  void addNeighborWall(Node newNeighbor, Wall wall){
    neighborToWall.put(newNeighbor, wall);
  }
  
  void addConnection(Wall newWall){
    connections.add(newWall);
  }
}

class NavMesh{   
   public ArrayList<PVector> reflexPoint; 
   public ArrayList<Wall> wallList;          
   public ArrayList<Node> convexPolygons;    
   HashMap<Integer, Node> wallMap = new HashMap<Integer, Node>();  
   
   void bake(Map map){
     //initializing
     ArrayList<Wall> Walls = map.walls;
     wallList = new ArrayList<Wall>();
     reflexPoint = new ArrayList<PVector>();         
     convexPolygons = new ArrayList<Node>();
     wallMap = new HashMap<Integer, Node>(); 
       
     Split(Walls);  //recursively splits the walls
         
     //each wall should have a list of connections, and a varying number of walls. the map should contain a listing for each of these corresponding walls
     for(int i = 0; i < convexPolygons.size(); i++){
       if(convexPolygons.get(i).connections.size() != convexPolygons.get(i).neighbors.size()){  //if they have the same number then every connecting wall should have its     
         for(int j = 0; j < convexPolygons.get(i).connections.size(); j++){  
               
           if(wallMap.containsKey(convexPolygons.get(i).connections.get(j).getID())){  
             convexPolygons.get(i).addNeighbor(wallMap.get(convexPolygons.get(i).connections.get(j).getID()));
             convexPolygons.get(i).addNeighborWall(wallMap.get(convexPolygons.get(i).connections.get(j).getID()), convexPolygons.get(i).connections.get(j));              }
           }                     
         }
       }   
      
     boolean printMap = false;
     print("\nThe Walls map is: " + wallMap);
     
    boolean printInfo = false;
    for(int l = 0; l < convexPolygons.size(); l++){
      print("\nInformation for polygon ID: " + convexPolygons.get(l).id);
      print("\n\t The connecting walls: ");
      for(int f = 0; f < convexPolygons.get(l).connections.size(); f++){
        print("\n\t\t wall has an ID of " + convexPolygons.get(l).connections.get(f).ID); 
      }
      print("\n\t The surrounding walls: ");
      for(int f = 0; f < convexPolygons.get(l).polygon.size(); f++){
        stroke(020);
      }
      print("\n\t The neighbor nodes: ");
      for(int f = 0; f < convexPolygons.get(l).neighbors.size(); f++){
        print(convexPolygons.get(l).neighbors.size());
        print("\n\t\t neighbor is: " + convexPolygons.get(l).neighbors.get(f).id + ", at point " + convexPolygons.get(l).neighbors.get(f).center);
        stroke(150);
      } 
    }
  }
   
  //Recursively split all polygons in the list until there is only convex ones left.
  void Split(ArrayList<Wall> polygons) {     
    int reflexIndex = isReflex(polygons);
    
    // If the polygon is already convex, add it to the convex polygon list
    if (reflexIndex == -1) {
        Node convexPolygon = new Node(convexPolygons.size(), polygons);

        for (Wall wall : polygons) {  
            if (wall.getID() != 0) {
                convexPolygon.addConnection(wall);
                int wallID = wall.getID();
                
                if (wallMap.containsKey(wallID)) {
                    Node neighborNode = wallMap.get(wallID);
                    convexPolygon.addNeighbor(neighborNode);
                    convexPolygon.addNeighborWall(neighborNode, wall);
                    wallMap.put(-wallID, convexPolygon);
                } else {
                    wallMap.put(-wallID, convexPolygon);
                }
            }
        }
        
        convexPolygons.add(convexPolygon);
        return;
    } 
    
    // Polygon is concave, find the reflex point and fix point
    PVector reflexPoint = polygons.get(reflexIndex).start;
    int fixPointIndex = findFixPoint(polygons, reflexIndex, reflexPoint);
    PVector fixPoint = polygons.get(fixPointIndex).start;

    // Ensure walls are ordered correctly for both polygons
    if (fixPointIndex < reflexIndex) {
        int temp = fixPointIndex;
        fixPointIndex = reflexIndex;
        reflexIndex = temp;
        reflexPoint = polygons.get(reflexIndex).start;
        fixPoint = polygons.get(fixPointIndex).start;
    }

    // Create walls for splitting
    Wall normalWall = new Wall(reflexPoint, fixPoint);
    normalWall.setID(wallList.size() + 1);
    Wall reverseWall = new Wall(fixPoint, reflexPoint);
    reverseWall.setID(-wallList.size() - 1);
    
    // Add the new wall to the list
    wallList.add(normalWall);   

    // Split walls into two new polygons
    ArrayList<Wall> frontWalls = new ArrayList<>();
    ArrayList<Wall> backWalls = new ArrayList<>();
    backWalls.add(reverseWall);

    for (int i = 0; i < polygons.size(); i++) {
        if (i >= reflexIndex && i < fixPointIndex) {
            backWalls.add(polygons.get(i));
        } else {
            frontWalls.add(polygons.get(i));
        }
    }
    
    frontWalls.add(reflexIndex, normalWall);
    
    // Recursively split the new polygons
    Split(frontWalls);
    Split(backWalls);
  }

  int findFixPoint(ArrayList<Wall> polygons, int reflexIndex, PVector reflexPoint) {
      int fixPointIndex = 0;
      int pointGap = 2;
      boolean reachable = false;
      boolean reflexive = true;
  
      // Iterate through all points to find a valid fix point
      while (!reachable || !reflexive) {
          fixPointIndex = (reflexIndex + pointGap) % polygons.size();
          PVector fixPoint = polygons.get(fixPointIndex).start;
          Wall testWall = new Wall(reflexPoint, fixPoint);
          
          reachable = canPlace(polygons, testWall);
          int previousIndex = (reflexIndex - 1 + polygons.size()) % polygons.size();
  
          // Check if the new wall makes a convex polygon
          reflexive = polygons.get(previousIndex).normal.dot(testWall.direction) < 0;
          pointGap++;
      }
  
      return fixPointIndex;
  }


  int isReflex(ArrayList<Wall> polygon){
    //loops trhough the list of polygons
    int reflexIndex = -1;
    for(int i = 0; i<polygon.size(); ++i){
      int next = (i+1) % polygon.size();
      if(polygon.get(i).normal.dot(polygon.get(next).direction) >= 0){
        print("\nReflex angle found");
        reflexIndex = next;
        //Convex = false;
        return reflexIndex;
      }
    }
    return reflexIndex;
  }
   
  //Making sure the walls won't collide
  Boolean canPlace (ArrayList<Wall> polygons, Wall TestWall){
    PVector From = PVector.add(TestWall.start, PVector.mult(TestWall.direction, 0.01));
    PVector To = PVector.add(TestWall.end, PVector.mult(TestWall.direction, -0.01));
    
    for(int n = 0; n<polygons.size(); n++){   
      if(polygons.get(n).crosses(From, To)){
        return false;
      }
    }
    return true;
  }
  
  //Printing for debugging
  void PrintWallCoords(ArrayList<Wall> Walls){
    for(int x = 0; x<Walls.size(); x++){
      print("\n Wall " + x + " starts at point " + Walls.get(x).start + " and ends at point " + Walls.get(x).end );
    }
  }
   
  //Contains a heuristic for the navigation algorithm, length of path, value for a*, and the current node.
 class frontierNode{
    float heuristic;
    float pathLength;
    float aValue;
    Node currentNode;
    frontierNode parentNode;
     
    frontierNode(Node currentNode, frontierNode parentNode, float pathLength, PVector target){
      this.currentNode = currentNode;
      this.parentNode = parentNode;
      this.pathLength = pathLength;
      this.heuristic = getPVectorDistance(currentNode.center, target);
      this.aValue = pathLength + heuristic;
    }
  }
   
  //Function to find the most optimal path using a*
  ArrayList<PVector> findPath(PVector boidPosition, PVector destinationPosition){
    ArrayList<PVector> path = new ArrayList<PVector>();
    
    //find which nodes boid and destination are in
    Node startNode = null;
    Node endNode = null;

    for(Node testNode: convexPolygons){     
      if(pointContained(boidPosition, testNode.polygon)){
        startNode = testNode;
        break;
      }
    }
     
    for(Node testNode: convexPolygons){
      if(pointContained(destinationPosition, testNode.polygon)){
        endNode = testNode;
        break;
      }
    }
     
    //call find node path
    if (startNode != endNode){
      ArrayList<Node> nodePath = new ArrayList<Node>();
      print("\nFinding path between nodes");
      nodePath = findNodePath(startNode, endNode);
       
      print("\n Number of nodes " + nodePath.size());
       
      for(int i = 0; i < nodePath.size() - 1; i++)
      {
        path.add(nodePath.get(i).neighborToWall.get(nodePath.get(i + 1)).center());
      }
    }
    
    path.add(destinationPosition);
    //might combine them all into one, tbd
    print("\n Number of points to travel through: " + path.size());
    return path;  
  }
   
  //Function to check the point we need to access
  boolean pointContained (PVector point, ArrayList<Wall> polygon){
    int crosses = 0;
    for(Wall wall: polygon)
    {
      if (wall.crosses(point, new PVector(point.x + 2*width, point.y))){
        crosses ++;
      }
    }  
    if((crosses % 2) == 0){
      return false;
    }
    return true;
  }
   
  //Function to find the proper node path
  ArrayList<Node> findNodePath(Node start, Node destination){
    print("\n Looking for path");
    ArrayList<frontierNode> frontierList = new ArrayList<frontierNode>();
    ArrayList<Node> previouslyExpandedList = new ArrayList<Node>();
     
    //Will add a new frontier every time it reaches a frontier.
    frontierList.add(new frontierNode(start, null, 0, destination.center));

    //While there are still frontiers, and we're not in the destination, it will keep looking and expanding horizons. It will also remove any older frontiers we've already been to
    while(frontierList.get(0).currentNode != destination){
      print("\n The current lowest node is at: " + frontierList.get(0).currentNode.center);
      for(int i = 0; i < frontierList.get(0).currentNode.neighbors.size(); i++){
        print("\n Adding Neighbors");
        float newPath = frontierList.get(0).pathLength + getPVectorDistance(frontierList.get(0).currentNode.center, frontierList.get(0).currentNode.neighbors.get(i).center);
        frontierList.add(new frontierNode(frontierList.get(0).currentNode.neighbors.get(i), frontierList.get(0), newPath, destination.center));
      }
      previouslyExpandedList.add(frontierList.get(0).currentNode);
      frontierList.remove(0);
      frontierList.sort(new FrontierCompare());
      while(previouslyExpandedList.contains(frontierList.get(0).currentNode)){
        frontierList.remove(0);
      }
      print("\n Done Sorting");
   }
     
    //Array list for the result. It will return a the optimal list.
    ArrayList<Node> result = new ArrayList<Node>();
    result.add(frontierList.get(0).currentNode);
      
    frontierNode parentNode = frontierList.get(0).parentNode;
      
    while(result.get(0) != start){
      result.add(0, parentNode.currentNode);
      parentNode = parentNode.parentNode;
    }
    return result;
  }
   
  //Function that compares frontiers. Whichever choice is the most optimal will have the higher value.
  class FrontierCompare implements Comparator<frontierNode>{
    int compare(frontierNode a, frontierNode b){
      print("\n Doing comparing things");
      if(a.aValue > b.aValue){
        return 1;
      } 
      else if(a.aValue < b.aValue){
        return -1;
      } 
      else
        return 0;
    }
  }
   
   
  void update(float dt)
  {
     draw();     
  }
   
  void draw()
  {
     /// use this to draw the nav mesh graph
     // Draw reflex points
    for (PVector point : reflexPoint) {
        stroke(0, 255, 0);
        circle(point.x, point.y, 20);
    }

    // Draw convex polygons and connections
    if (convexPolygons != null) {
        for (Node polygonNode : convexPolygons) {
            stroke(255, 0, 0);
            circle(polygonNode.center.x, polygonNode.center.y, 5);

            // Draw connections between neighbors
            for (Wall connection : polygonNode.connections) {
                stroke(255, 0, 0);
                line(connection.start.x, connection.start.y, connection.end.x, connection.end.y);
            }
        }
    } 
  }
}
