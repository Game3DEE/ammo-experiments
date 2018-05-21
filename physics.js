// Physics variables
var collisionConfiguration;
var dispatcher;
var broadphase;
var solver;
/*export*/var physicsWorld;

function initPhysics(gravity) {
  // Physics configuration
  collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  dispatcher = new Ammo.btCollisionDispatcher( collisionConfiguration );
  broadphase = new Ammo.btDbvtBroadphase();
  solver = new Ammo.btSequentialImpulseConstraintSolver();
  physicsWorld = new Ammo.btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );
  physicsWorld.setGravity( new Ammo.btVector3( 0, gravity || -9.82, 0 ) );
}

