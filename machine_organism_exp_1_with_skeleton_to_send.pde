import processing.opengl.*;
import igeo.*;
int c=0;

IGeometry geo;

void setup(){
  size(1920, 1080, IG.GL);
  
  IG.open("geometry.3dm");
  geo = IG.geometry(0);
  geo.del();

  //IG.fill();
  IG.left();
  
  
  IConfig.syncDrawAndDynamics=true;
  IG.bg(0);
  new ClockStackAgent(new IVec(0,0,0), new IVec(10,0,0), new IVec(0,0,1), null, null).clr(0);
}


class Orientation{
  IVec dir, nml; // as xyz coordinates system, dir corresponds to Y and nml corresponds to Z
  boolean righthand; // right hand coordinates system or not
  IVec translate; // just to implement jumping behavior
  Orientation(IVec d, IVec n, boolean righthandsys){ dir=d; nml=n; righthand=righthandsys; }
  Orientation(IVec d, IVec n){ this(d,n,true); }
  Orientation(Orientation o){ dir=o.dir.cp(); nml=o.nml.cp(); righthand=o.righthand; }
  Orientation cp(){ return new Orientation(this); }
  IVec dir(){ return dir.cp(); }
  IVec front(){ return dir(); }
  IVec back(){ return dir().neg(); }
  IVec nml(){ return nml.cp(); }
  IVec up(){ return nml(); }
  IVec down(){ return nml().neg(); }
  IVec side(){ if(righthand){ return dir.cross(nml); }else{ return nml.cross(dir); } }
  IVec right(){ return side(); }
  IVec left(){ if(righthand){ return nml.cross(dir); }else{ return dir.cross(nml); } }
  Orientation rot(double ang){ dir.rot(nml, ang); return this; }
  Orientation rot(IVec ax, double ang){ dir.rot(ax,ang); nml.rot(ax,ang); return this; }
  Orientation pitch(double ang){
    IVec ax = dir.cross(nml); 
    dir.rot(ax, ang); nml.rot(ax, ang);
    return this;
  }
  Orientation yaw(double ang){ dir.rot(nml, ang); return this; }
  Orientation roll(double ang){ nml.rot(dir, ang); return this; }
  Orientation ref(IVec refNml){
    dir.ref(refNml); nml.ref(refNml); righthand = !righthand;
    return this;
  }
  Orientation flip(){ dir.flip(); righthand = !righthand; return this; }// flip front/back
  Orientation flipNml(){ nml.flip(); righthand = !righthand; return this; }// flip up/down
  Orientation flipSide(){ righthand = !righthand; return this; }// flip left/right
  Orientation mul(double v){ dir.mul(v); return this; }
  Orientation div(double v){ dir.div(v); return this; }
  
  Orientation add(Orientation o){ dir.add(o.dir); nml.add(o.nml()); return this; }
  Orientation add(Orientation o, double f){ dir.add(o.dir, f); nml.add(o.nml(), f); return this; }
  Orientation sum(Orientation o, double f){ dir.mul(1-f).add(o.dir, f); nml.mul(1-f).add(o.nml(), f); return this; }
  Orientation mid(Orientation o){ return sum(o,0.5); }
  
  Orientation translate(IVec t){ return jump(t); }
  Orientation jump(IVec move){ translate=move; return this; }
}

class Attribute extends IAttribute{
  int delay=0;
  boolean noCollision=false;
  boolean noGeometry=false;
  Attribute(){ super(); }
  Attribute(IAttribute at){ super(at); }
  Attribute(Attribute at){ 
    super(at);
    delay = at.delay;
    noCollision = at.noCollision;
    noGeometry = at.noGeometry;
  }
  Attribute cp(){
    return new Attribute(this); 
  }
  Attribute delay(int d){ delay = d; return this; }
  Attribute noCollision(){ noCollision=true; return this; }
  Attribute collision(){ noCollision=false; return this; }
  Attribute noGeometry(){ noGeometry=true; return this; }
  Attribute geometry(){ noGeometry=false; return this; }
  
}


class ClockStackAgent extends IAgent{
  final double threshold = 1; // collision threshold
  IVec pos, pos2, prevPos;
  Orientation orient, prevOrient;
  int[] clocks;
  
  boolean isColliding = false, isStopped = false;
  ArrayList< IVec > pts;
  ArrayList< Orientation > nextOrient;
  ArrayList< int[] > nextClocks;
  ArrayList< Attribute > nextAttr;
  IBounds bounds;
  int delayCount;
  
  ClockStackAgent(IVec p, Orientation o, int[] clok, IVec prevP, Orientation prevO){
    pos = p;
    orient = o;
    clocks = clok;
    prevPos = prevP;
    prevOrient = prevO;
    delayCount=0;
  }
  
  ClockStackAgent(IVec p, IVec d, IVec n, int[] clok, IVec prevP, Orientation prevO){
    pos = p;
    if(d.isParallel(n)){
      if(!n.isParallel(IG.zaxis)) n = new IVec(0,0,1);
      else n = new IVec(0,1,0);
    } 
    if(d.dot(n)!=0) n = d.cross(n).icross(d);
    orient = new Orientation(d,n);
    clocks = clok;
    prevPos = prevP;
    prevOrient = prevO;
    delayCount=0;
  }
  
  ClockStackAgent(IVec p, IVec d, IVec n, IVec prevP, Orientation prevO){
    this(p,d,n,new int[0], prevP, prevO);
  }
  
  ClockStackAgent(IVec p, IVec d, IVec prevP, Orientation prevO){
    this(p,d,new IVec(0,0,1),null, prevP, prevO);
  }
  
  IVec pos2(){
    if(pos2==null) pos2 = pos.cp(orient.dir);
    return pos2;
  }
  
  IAttribute defaultAttribute(){ return new Attribute(); }
  
  ClockStackAgent delay(int d){
    IAttribute attr = attr();
    if(attr==null){ attr = defaultAttribute(); attr(attr); } 
    ((Attribute)attr).delay(d); 
    return this; 
  }
  ClockStackAgent noCollision(){ 
    IAttribute attr = attr();
    if(attr==null){ attr = defaultAttribute(); attr(attr); } 
    ((Attribute)attr).noCollision(); 
    return this; 
  }
  ClockStackAgent collision(){
    IAttribute attr = attr();
    if(attr==null){ attr = defaultAttribute(); attr(attr); } 
    ((Attribute)attr).collision(); 
    return this; 
  }
  
  boolean isDelayed(){
    if(attr()==null) return false;
    if(((Attribute)attr()).delay<=delayCount) return true;
    return false;
  }
  
  int delayedTime(){
    if(attr()==null) return time();
    return delayCount - ((Attribute)attr()).delay;
  }
  
  boolean isCollidable(){
    if(attr()==null) return true;
    if(((Attribute)attr()).noCollision) return false;
    if(((Attribute)attr()).delay <= delayCount) return true;
    return false;
  }
  
  void interact(ArrayList< IDynamics > agents){
    if(threshold > 0 && !isStopped && isCollidable()){
      IVec pt2 = pos2();
      for(int i=0; i < agents.size() && !isColliding; i++){
        if(agents.get(i) instanceof ClockStackAgent){
          ClockStackAgent a = (ClockStackAgent)agents.get(i);
          if(a==this){ // check self collision
            for(int j=0; pts!=null && j < pts.size()-2 && !isColliding; j++){ // exclude last segment
              if(IVec.isSegCloserThan(pos, pt2, pts.get(j), pts.get(j+1), threshold)){
                isColliding = true;
              }
            }
          }
          else if(a.delayedTime() >= 0 || !a.isColliding){ // a!=this
            if(a.bounds!=null && bounds!=null){
              IBounds newbounds = bounds.cp();
              newbounds.compare(pt2);
              if(!newbounds.isCloserThan(a.bounds,threshold)){
                continue;
              }
            }
            IVec apt2 = a.pos2();
            if(IVec.isSegCloserThan(pos, pt2, a.pos, apt2, threshold) && (!pos.eq(a.pos) || pt2.eq(apt2)) ){
              isColliding = true;
            }
            for(int j=0; a.pts!=null && j < a.pts.size() && !isColliding; j++){
              IVec apt3 = a.pos;
              if(j < a.pts.size()-1) apt3 = a.pts.get(j+1);
              if(IVec.isSegCloserThan(pos, pt2, a.pts.get(j), apt3, threshold)
                && (!pos.eq(a.pos) || pt2.eq(apt2) || j < a.pts.size()-1) ){
                if(delayedTime()>0 || !pos.isOnSegment(a.pts.get(j),apt3)){ // exclude if it came from that line
                  isColliding = true;
                }
              }
            }
          }
        }
      }
    }
  }
  
  int clock(int i){
    if(i >= clocks.length) return 0;
    return clocks[i];
  }
  
  Attribute next(int incrementClock){
    return next(orient, incrementClock);
  }
  
  Attribute next(Orientation o, int incrementClock){
    if(nextOrient==null){
      nextOrient = new ArrayList< Orientation >();
      nextClocks = new ArrayList< int[] >();
      nextAttr = new ArrayList< Attribute >();
    }
    nextOrient.add(o);
    int[] clocks2 = new int[incrementClock+1 > clocks.length?incrementClock+1:clocks.length];
    for(int i=0; i < clocks2.length; i++){
      if(i < incrementClock) clocks2[i] = 0;
      else if(i < clocks.length) clocks2[i] = clocks[i];
    }
    clocks2[incrementClock]++;
    nextClocks.add(clocks2);
    Attribute attr = null;
    if(attr()==null) attr = new Attribute();
    else{
      IAttribute at = attr();
      if(at instanceof Attribute) attr = ((Attribute)at).cp(); 
      else attr = new Attribute(at);
    }   
    nextAttr.add(attr);
    return attr;
  }
  
  void generate(){  
    if(nextOrient==null || nextOrient.size()==0){
      isStopped=true;
      return;
    }
    for(int i=0; i < nextOrient.size(); i++){
      Orientation orient2 = nextOrient.get(i);
      if(i > 0 || orient2.translate!=null){
        if(orient2.translate!=null){
          IVec pos2 = pos.cp(orient2.translate);
          orient2.translate = null; // jump happens only once
          new ClockStackAgent(pos2, orient2, nextClocks.get(i), null, null).attr(nextAttr.get(i));
          if(i==0) isStopped=true;
        }
        else{
          new ClockStackAgent(pos2(), orient2, nextClocks.get(i), pos.cp(), orient.cp()).attr(nextAttr.get(i));
        }
      }
      else{
        if(pts==null){
          pts = new ArrayList< IVec >();
          bounds = new IBounds(pos);
          bounds.compare(pos2);
        }
        if(pts.size() > 1 && pts.get(pts.size()-1).isOnLine(pos, pts.get(pts.size()-2))){
          pts.set(pts.size()-1, pos);
        }
        else{ pts.add(pos); } // past points
        prevPos = pos;
        pos = pos2();
        prevOrient = orient;
        orient = orient2;
        clocks = nextClocks.get(i);
        attr(nextAttr.get(i));
        bounds.compare(pos2); // next point
        delayCount=0;
      }
    }
    pos2 = null; // reset pos2
    nextOrient=null;
    nextClocks=null;
    nextAttr=null;
  }
  
  IVec dir(){ return orient.dir(); }
  IVec front(){ return orient.front(); }
  IVec back(){ return orient.back(); }
  IVec nml(){ return orient.nml(); }
  IVec up(){ return orient.up(); }
  IVec down(){ return orient.down(); }
  IVec right(){ return orient.right(); }
  IVec left(){ return orient.left(); }
  IVec side(){ return orient.side(); }
  
  IVec prevDir(){ if(prevOrient==null) return null; return prevOrient.dir(); }
  IVec prevFront(){ if(prevOrient==null) return null; return prevOrient.front(); }
  IVec prevBack(){ if(prevOrient==null) return null; return prevOrient.back(); }
  IVec prevNml(){ if(prevOrient==null) return null; return prevOrient.nml(); }
  IVec prevUp(){ if(prevOrient==null) return null; return prevOrient.up(); }
  IVec prevDown(){ if(prevOrient==null) return null; return prevOrient.down(); }
  IVec prevRight(){ if(prevOrient==null) return null; return prevOrient.right(); }
  IVec prevLeft(){ if(prevOrient==null) return null; return prevOrient.left(); }
  IVec prevSide(){ if(prevOrient==null) return null; return prevOrient.side(); }
  
  // transformation methods
  Orientation rot(double angle){ return orient.cp().rot(angle); }
  Orientation rot(IVec axis, double angle){ return orient.cp().rot(axis,angle); }
  Orientation pitch(double angle){ return orient.cp().pitch(angle); }
  Orientation yaw(double angle){ return orient.cp().yaw(angle); }
  Orientation roll(double angle){ return orient.cp().roll(angle); }
  Orientation mul(double factor){ return orient.cp().mul(factor); }
  Orientation div(double factor){ return orient.cp().div(factor); }
  Orientation ref(IVec axis){ return orient.cp().ref(axis); }
  Orientation flip(){ return orient.cp().flip(); }
  Orientation flipNml(){ return orient.cp().flipNml(); }
  Orientation flipSide(){ return orient.cp().flipSide(); }
  
  Orientation jump(IVec move){
    return orient.cp().jump(move);
  }
  Orientation jump(double x, double y, double z){
    return orient.cp().jump(new IVec(x,y,z));
  }
  
  void update(){
    if(isStopped){
      return;
    }
    if(attr()==null || ((Attribute)attr()).delay<=delayCount){
      if(isColliding){
        if(attr()==null && time()==0 || 
          ((Attribute)attr()).delay==time()){ del(); }
        else isStopped=true;
        return;
      }
      pos2 = pos2();
      // make geometry
      makeGeometry();
      rules();
      generate();
      delayCount=0;
    }
    else{
      delayCount++;
    }
  }
  
  IPoint makePoint(){
    return new IPoint(pos).attr(attr());
  }
  ICurve makeLine(){ // extrusions for body of the organism
    double height = dir().len()*3.5 * (sin(IG.time()*0.2) + 2);
  IG.extrude(new IVec[]{ pos.cp().z(-IG.time()), pos2.cp().z(-IG.time()) }, new IVec(0,0, height)).attr(attr());
    //return new ICurve(pos, pos2).attr(attr());
    return null;
  }
  ISurface makeSurface(){
    IVec[][] pts = new IVec[2][2];
    double len = orient.dir().len()/2;
    IVec side = right().cp().len(len);
    pts[0][0] = pos.cp().add(side);
    pts[0][1] = pos.cp().sub(side);
    pts[1][0] = pos2.cp().add(side);
    pts[1][1] = pos2.cp().sub(side);
    return new ISurface(pts).attr(attr());
  }
  IBox makeBox(){ // code for the skeleton system
    IVec[][][] pts = new IVec[2][2][2];
    double len = orient.dir().len()/2;
    double t =0.9;
    IVec floor = new IVec (0,0,-IG.time()*1*(sin(IG.time()*0.01) + 2)/3);
    IVec side = right().cp().len((len*1)-0.01*IG.time());
    IVec up = up().cp().len(height*0.001);
    pts[0][0][0] = pos.cp().add(side).sub(up.mul(t)).add(floor);
    pts[0][1][0] = pos.cp().sub(side).sub(up.mul(t)).add(floor);
    pts[1][0][0] = pos2.cp().add(side).sub(up.mul(t)).add(floor);
    pts[1][1][0] = pos2.cp().sub(side).sub(up.mul(t)).add(floor);
    pts[0][0][1] = pos.cp().add(side).add(up.mul(t)).add(floor);
    pts[0][1][1] = pos.cp().sub(side).add(up.mul(t)).add(floor);
    pts[1][0][1] = pos2.cp().add(side).add(up.mul(t)).add(floor);
    pts[1][1][1] = pos2.cp().sub(side).add(up.mul(t)).add(floor);
    return (IBox)new IBox(pts).attr(attr());
  }
  ISphere makeSphere(){
    IVec mid = pos.mid(pos2);
    double len = pos.dist(pos2);
    return (ISphere)new ISphere(mid, len/2).attr(attr());
  }
  ICurve makeTangentCurve(){
    if(prevPos!=null && prevOrient!=null){ 
      IVec m1 = prevPos.mid(pos);
      IVec m2 = pos.mid(pos2);
      double len = orient.dir().len()/2;
      if(!prevOrient.dir.isParallel(orient.dir) || !prevOrient.nml.isParallel(orient.nml)){
        IVec[] pts = new IVec[3];
        Orientation ori = orient.cp().mid(prevOrient);
        pts[0] = m1;
        pts[1] = pos;
        pts[2] = m2;
        return new ICurve(pts, 2).attr(attr()); 
      }
      return new ICurve(m1, m2).attr(attr());
    }
    return null;
  }
  ISurface makeTangentSurface(){
    if(prevPos!=null && prevOrient!=null){ 
      IVec m1 = prevPos.mid(pos);
      IVec m2 = pos.mid(pos2);
      double len = orient.dir().len()/2;
      if(!prevOrient.dir.isParallel(orient.dir) || !prevOrient.nml.isParallel(orient.nml)){
        IVec[][] pts = new IVec[3][2];
        Orientation ori = orient.cp().mid(prevOrient);
        pts[0][0] = m1.cp(prevRight().cp().len(len));
        pts[1][0] = pos.cp(ori.right().cp().len(len));
        pts[2][0] = m2.cp(right().cp().len(len));
        pts[0][1] = m1.cp(prevLeft().cp().len(len));
        pts[1][1] = pos.cp(ori.left().cp().len(len));
        pts[2][1] = m2.cp(left().cp().len(len));
        return new ISurface(pts, 2, 1).attr(attr()); 
      }
      return new ISurface(m1.cp(prevRight().cp().len(len)),
                        m1.cp(prevLeft().cp().len(len)),
                        m2.cp(left().cp().len(len)),
                        m2.cp(right().cp().len(len))).attr(attr());
    }
    return null;
  }
  
  void makeGeometry(){
    if(attr()!=null && ((Attribute)attr()).noGeometry) return;
    //makePoint();
    makeLine(); // changed this definition to generate extrusions
    //makeSurface();
    makeBox().clr(200); //changed this definition for the skeletal system
    //makeSphere();
    //makeTangentCurve();
    //makeTangentSurface();
  }
   
  // update rules
  void rules(){
    
  //if (keyPressed) { 
    //IG.fill();
    if (clock(3)==0) { 
     if(clock(1)==0) {
      if(clock(0)==5) {
        next(rot(PI/3), 1);
      }
      else{ next(mul(0.99),0).clr(0+IG.time()*0.0025,0.7-IG.time()*0.0025,0+IG.time()*0.0025); }
    }
    else{
      if(clock(0)==13){
        next(rot(PI/2), 1);
        
       //column like structures generation
       if(IRand.pct(40)& c<2000){
          geo.cp().scale(10-IG.time()*0.02).add(pos.cp().z(-IG.time())).clr(1.0,1.0,1.0);
          c++;
        }
        
        if (clock(2) == 1) {
        
          next(rot(0).mul(0.96),3).clr(1.0,0,1.0);}
         }
    
      else if(clock(0)==4){
        //if (IRand.pct(70)) {
        next(rot(-PI/3).flipNml(), 2).clr(0,0,1.);
        next(0);
      }
      else{ next(mul(0.995), 0).clr(0+IG.time()*0.0025,0.7-IG.time()*0.0025,0+IG.time()*0.0025); }
    }
    }
    
    else if (clock(3)>0){
      //if (IRand.pct(50)) {
     if(clock(3)==1){
      if(clock(0)>=2 && IRand.pct(20)){
        next(4);
      }
      else{ next(0); }
    }
    else if(clock(3)==1){
      if(clock(1)==0){
        next(rot(-PI/3), 3);
      }
      if(clock(1)==0){
        next(rot(PI/3), 2);
      }
      else{ next(rot(PI/3), 1); }
    }
    else{ next(0); } 
      }
  //}
  }
  }
    void draw() {
  saveFrame("specie1####.png");
}
