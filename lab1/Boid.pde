/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
     this.position = position;
  }
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
   Crumb[] crumbs = {};
   int last_crumb;
   float acceleration;
   float rotational_acceleration;
   KinematicMovement kinematic;
   PVector target;
   int currentIndex = 0;
   ArrayList<PVector> waypoints;
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }

   void update(float dt)
   {
     if (target != null)
     {  
        //Implement seek here
        
        //Finding the distance between the boid and the target
        PVector desired = PVector.sub(target, kinematic.getPosition());
        //Keep the distance in a variable
        float d = desired.mag();
        
        
        //Check if the boid is close enough to the target already
        if(d > 15){
          //Get only the direction
          desired.normalize();
          
          //Calculate the angle the boid should be facing and 
          //Find the difference between current boid angle and desired angle
          //make sure its between -pi and pi
          float desiredHeading = atan2(desired.y, desired.x);
          float difference = desiredHeading - this.kinematic.getHeading();    
          difference = atan2(sin(difference), cos(difference)); 
          
          if(abs(difference) > HALF_PI/2){
            float rSpeed = constrain(difference, -this.kinematic.max_rotational_speed, this.kinematic.max_rotational_speed);  
            this.kinematic.increaseSpeed(0, rSpeed);
          }
          else {
            float approachSpeed = map(d, 15, 100, 0.1, this.kinematic.max_speed);
            approachSpeed = constrain(approachSpeed, 0, this.kinematic.max_speed);
            float rSpeed = constrain(difference, -this.kinematic.max_rotational_speed, this.kinematic.max_rotational_speed);  
            
            this.kinematic.increaseSpeed(approachSpeed, rSpeed*2);
          }
          
        }
        else {
          //Stop the boid when it reaches the destination
          this.kinematic.increaseSpeed(-(this.kinematic.getSpeed()), -(this.kinematic.getRotationalVelocity())); 
          
          if(waypoints != null && currentIndex < waypoints.size() - 1){
            currentIndex++;
            this.target = waypoints.get(currentIndex);
          }
       }
     }
     
     // place crumbs, do not change     
     if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
     {
        this.last_crumb = millis();
        this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
        if (this.crumbs.length > MAX_CRUMBS)
           this.crumbs = (Crumb[])subset(this.crumbs, 1);
     }
     
     // do not change
     this.kinematic.update(dt);
     
     draw();
   
 }
   
   void draw()
   {
     for (Crumb c : this.crumbs)
     {
       c.draw();
     }
     
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     // front
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
     // left
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
     // right
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   } 
   
   void seek(PVector target)
   {
      ArrayList<PVector> path = nm.findPath(billy.kinematic.position, target);
      this.target = path.get(0);
      this.waypoints = path;
      
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      this.waypoints = waypoints;
      this.currentIndex = 0;
      this.target = waypoints.get(currentIndex);
   }
 }
