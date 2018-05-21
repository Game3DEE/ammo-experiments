var chassisMesh;

// position, target: vec3
// trackingSpeed, safeDistance: number
var camv = new THREE.Vector3();
var ret_pos = new THREE.Vector3();
function trackTarget(position, target, trackingSpeed, safeDistance) {
  camv.subVectors(target, position);
  let fdist = camv.length();
  let motionv = camv.clone();

  motionv.normalize();
  let v = trackingSpeed * (1.0 / (1.0 / (fdist - safeDistance)));
  motionv.multiplyScalar(v);

  let ret_pos = new THREE.Vector3();
  if (fdist > safeDistance) {
    ret_pos.addVectors(position, motionv);
  } else if (fdist < safeDistance) {
    motionv.copy(camv);
    motionv.normalize();
    let v = trackingSpeed * (1.0 / (1.0 / (Math.abs(fdist - safeDistance))));
    motionv.multiplyScalar(v);
    ret_pos.subVectors(position, motionv);
  } else {
    ret_pos.set( position.x, position.y + motionv.z, position.z );
  }

  return ret_pos;
}


Ammo().then(function(Ammo) {

  // Detects webgl
  if ( ! Detector.webgl ) {
    Detector.addGetWebGLMessage();
    document.getElementById( 'container' ).innerHTML = "";
  }

  // - Global variables -
  var DISABLE_DEACTIVATION = 4;
  var TRANSFORM_AUX = new Ammo.btTransform();
  var ZERO_QUATERNION = new THREE.Quaternion(0, 0, 0, 1);

  // Graphics variables
  var container, stats, speedometer;
  var camera, controls, scene, renderer;
  var terrainMesh, texture;
  var clock = new THREE.Clock();
  var materialDynamic, materialStatic, materialInteractive;

  var syncList = [];
  var time = 0;
  var objectTimePeriod = 3;
  var timeNextSpawn = time + objectTimePeriod;
  var maxNumObjects = 30;

  // Keybord actions
  var actions = {};
  var keysActions = {
    "KeyW":'acceleration',
    "KeyS":'braking',
    "KeyA":'left',
    "KeyD":'right'
  };

  // - Functions -

  function initGraphics() {

    container = document.getElementById( 'container' );
    speedometer = document.getElementById( 'speedometer' );

    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 0.2, 2000 );
    camera.position.x = -4.84;
    camera.position.y = 4.39;
    camera.position.z = -35.11;
    camera.lookAt( new THREE.Vector3( 0.33, -0.40, 0.85 ) );
    controls = new THREE.OrbitControls( camera );

    renderer = new THREE.WebGLRenderer({antialias:true});
    renderer.setClearColor( 0x2266FF );
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( window.innerWidth, window.innerHeight );

    var ambientLight = new THREE.AmbientLight( 0x333333 );
    scene.add( ambientLight );

    var dirLight = new THREE.DirectionalLight( 0xFFF8E8, 1 );
    dirLight.position.set( 10, 10, 5 );
    scene.add( dirLight );

    materialDynamic = new THREE.MeshPhongMaterial( { color:0xfca400 } );
    materialStatic = new THREE.MeshPhongMaterial( { color:0x999999 } );
    materialInteractive=new THREE.MeshPhongMaterial( { color:0x990000 } );

    container.innerHTML = "";

    container.appendChild( renderer.domElement );

    stats = new Stats();
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.top = '0px';
    container.appendChild( stats.domElement );

    window.addEventListener( 'resize', onWindowResize, false );
    window.addEventListener( 'keydown', keydown);
    window.addEventListener( 'keyup', keyup);
  }

  function onWindowResize() {

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    renderer.setSize( window.innerWidth, window.innerHeight );

  }

  function animate() {
    requestAnimationFrame( animate );
    var dt = clock.getDelta();
    for (var i = 0; i < syncList.length; i++)
      syncList[i](dt);
    physicsWorld.stepSimulation( dt, 10 );
    controls.update( dt );

    if (chassisMesh) {
      camera.position.copy(
        trackTarget(camera.position, chassisMesh.position, dt * 5, 4.0)
      );
  /*
      // keep camera above ground
      if (camera.position.y < GROUND_LEVEL + 50.0) {
        camera.position.y = GROUND_LEVEL + 50.0;
      }*/
      camera.lookAt(chassisMesh.position);
      camera.updateProjectionMatrix();
    }

    renderer.render( scene, camera );
    time += dt;
    stats.update();
  }

  function keyup(e) {
    if(keysActions[e.code]) {
      actions[keysActions[e.code]] = false;
      e.preventDefault();
      e.stopPropagation();
      return false;
    }
  }
  function keydown(e) {
    if(keysActions[e.code]) {
      actions[keysActions[e.code]] = true;
      e.preventDefault();
      e.stopPropagation();
      return false;
    }
  }

  function createBox(pos, quat, w, l, h, mass, friction) {
    var material = mass > 0 ? materialDynamic : materialStatic;
    var shape = new THREE.BoxGeometry(w, l, h, 1, 1, 1);
    var geometry = new Ammo.btBoxShape(new Ammo.btVector3(w * 0.5, l * 0.5, h * 0.5));

    if(!mass) mass = 0;
    if(!friction) friction = 1;

    var mesh = new THREE.Mesh(shape, material);
    mesh.position.copy(pos);
    mesh.quaternion.copy(quat);
    scene.add( mesh );

    var transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
    transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
    var motionState = new Ammo.btDefaultMotionState(transform);

    var localInertia = new Ammo.btVector3(0, 0, 0);
    geometry.calculateLocalInertia(mass, localInertia);

    var rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, geometry, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);

    body.setFriction(friction);
    //body.setRestitution(.9);
    //body.setDamping(0.2, 0.2);

    physicsWorld.addRigidBody( body );

    if (mass > 0) {
      body.setActivationState(DISABLE_DEACTIVATION);
      // Sync physics and graphics
      function sync(dt) {
        var ms = body.getMotionState();
        if (ms) {
          ms.getWorldTransform(TRANSFORM_AUX);
          var p = TRANSFORM_AUX.getOrigin();
          var q = TRANSFORM_AUX.getRotation();
          mesh.position.set(p.x(), p.y(), p.z());
          mesh.quaternion.set(q.x(), q.y(), q.z(), q.w());
        }
      }

      syncList.push(sync);
    }
  }

  function createWheelMesh(radius, width) {
    var t = new THREE.CylinderGeometry(radius, radius, width, 24, 1);
    t.rotateZ(Math.PI / 2);
    var mesh = new THREE.Mesh(t, materialInteractive);
    mesh.add(new THREE.Mesh(new THREE.BoxGeometry(width * 1.5, radius * 1.75, radius*.25, 1, 1, 1), materialInteractive));
    scene.add(mesh);
    return mesh;
  }

  function createChassisMesh(w, l, h) {
    var shape = new THREE.BoxGeometry(w, l, h, 1, 1, 1);
    chassisMesh = new THREE.Mesh(shape, materialInteractive);
    scene.add(chassisMesh);
    return chassisMesh;
  }

  function createVehicle(pos, quat) {

    // Vehicle contants
    var chassisWidth = 1.8;
    var chassisHeight = .6;
    var chassisLength = 4;
    var massVehicle = 800;
    var wheelAxisPositionBack = -1;
    var wheelRadiusBack = .4;
    var wheelWidthBack = .3;
    var wheelHalfTrackBack = 1;
    var wheelAxisHeightBack = .3;
    var wheelAxisFrontPosition = 1.7;
    var wheelHalfTrackFront = 1;
    var wheelAxisHeightFront = .3;
    var wheelRadiusFront = .35;
    var wheelWidthFront = .2;

    var friction = 1000;
    var suspensionStiffness = 20.0;
    var suspensionDamping = 2.3;
    var suspensionCompression = 4.4;
    var suspensionRestLength = 0.6;
    var rollInfluence = 0.2;

    var steeringIncrement = .04;
    var steeringClamp = .5;
    var maxEngineForce = 2000;
    var maxBreakingForce = 100;

    // Chassis
    var geometry = new Ammo.btBoxShape(new Ammo.btVector3(chassisWidth * .5, chassisHeight * .5, chassisLength * .5));
    var transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
    transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
    var motionState = new Ammo.btDefaultMotionState(transform);
    var localInertia = new Ammo.btVector3(0, 0, 0);
    geometry.calculateLocalInertia(massVehicle, localInertia);
    var body = new Ammo.btRigidBody(new Ammo.btRigidBodyConstructionInfo(massVehicle, motionState, geometry, localInertia));
    body.setActivationState(DISABLE_DEACTIVATION);
    physicsWorld.addRigidBody(body);
    var chassisMesh = createChassisMesh(chassisWidth, chassisHeight, chassisLength);
    // Raycast Vehicle
    var engineForce = 0;
    var vehicleSteering = 0;
    var breakingForce = 0;
    var tuning = new Ammo.btVehicleTuning();
    var rayCaster = new Ammo.btDefaultVehicleRaycaster(physicsWorld);
    var vehicle = new Ammo.btRaycastVehicle(tuning, body, rayCaster);
    vehicle.setCoordinateSystem(0, 1, 2);
    physicsWorld.addAction(vehicle);

    // Wheels
    var FRONT_LEFT = 0;
    var FRONT_RIGHT = 1;
    var BACK_LEFT = 2;
    var BACK_RIGHT = 3;
    var wheelMeshes = [];
    var wheelDirectionCS0 = new Ammo.btVector3(0, -1, 0);
    var wheelAxleCS = new Ammo.btVector3(-1, 0, 0);

    function addWheel(isFront, pos, radius, width, index) {

      var wheelInfo = vehicle.addWheel(
          pos,
          wheelDirectionCS0,
          wheelAxleCS,
          suspensionRestLength,
          radius,
          tuning,
          isFront);

      wheelInfo.set_m_suspensionStiffness(suspensionStiffness);
      wheelInfo.set_m_wheelsDampingRelaxation(suspensionDamping);
      wheelInfo.set_m_wheelsDampingCompression(suspensionCompression);
      wheelInfo.set_m_frictionSlip(friction);
      wheelInfo.set_m_rollInfluence(rollInfluence);

      wheelMeshes[index] = createWheelMesh(radius, width);
    }

    addWheel(true, new Ammo.btVector3(wheelHalfTrackFront, wheelAxisHeightFront, wheelAxisFrontPosition), wheelRadiusFront, wheelWidthFront, FRONT_LEFT);
    addWheel(true, new Ammo.btVector3(-wheelHalfTrackFront, wheelAxisHeightFront, wheelAxisFrontPosition), wheelRadiusFront, wheelWidthFront, FRONT_RIGHT);
    addWheel(false, new Ammo.btVector3(-wheelHalfTrackBack, wheelAxisHeightBack, wheelAxisPositionBack), wheelRadiusBack, wheelWidthBack, BACK_LEFT);
    addWheel(false, new Ammo.btVector3(wheelHalfTrackBack, wheelAxisHeightBack, wheelAxisPositionBack), wheelRadiusBack, wheelWidthBack, BACK_RIGHT);

    // Sync keybord actions and physics and graphics
    function sync(dt) {

      var speed = vehicle.getCurrentSpeedKmHour();

      speedometer.innerHTML = (speed < 0 ? '(R) ' : '') + Math.abs(speed).toFixed(1) + ' km/h';

      breakingForce = 0;
      engineForce = 0;

      if (actions.acceleration) {
        if (speed < -1)
          breakingForce = maxBreakingForce;
        else engineForce = maxEngineForce;
      }
      if (actions.braking) {
        if (speed > 1)
          breakingForce = maxBreakingForce;
        else engineForce = -maxEngineForce / 2;
      }
      if (actions.left) {
        if (vehicleSteering < steeringClamp)
          vehicleSteering += steeringIncrement;
      }
      else {
        if (actions.right) {
          if (vehicleSteering > -steeringClamp)
            vehicleSteering -= steeringIncrement;
        }
        else {
          if (vehicleSteering < -steeringIncrement)
            vehicleSteering += steeringIncrement;
          else {
            if (vehicleSteering > steeringIncrement)
              vehicleSteering -= steeringIncrement;
            else {
              vehicleSteering = 0;
            }
          }
        }
      }

      vehicle.applyEngineForce(engineForce, BACK_LEFT);
      vehicle.applyEngineForce(engineForce, BACK_RIGHT);

      vehicle.setBrake(breakingForce / 2, FRONT_LEFT);
      vehicle.setBrake(breakingForce / 2, FRONT_RIGHT);
      vehicle.setBrake(breakingForce, BACK_LEFT);
      vehicle.setBrake(breakingForce, BACK_RIGHT);

      vehicle.setSteeringValue(vehicleSteering, FRONT_LEFT);
      vehicle.setSteeringValue(vehicleSteering, FRONT_RIGHT);

      var tm, p, q, i;
      var n = vehicle.getNumWheels();
      for (i = 0; i < n; i++) {
        vehicle.updateWheelTransform(i, true);
        tm = vehicle.getWheelTransformWS(i);
        p = tm.getOrigin();
        q = tm.getRotation();
        wheelMeshes[i].position.set(p.x(), p.y(), p.z());
        wheelMeshes[i].quaternion.set(q.x(), q.y(), q.z(), q.w());
      }

      tm = vehicle.getChassisWorldTransform();
      p = tm.getOrigin();
      q = tm.getRotation();
      chassisMesh.position.set(p.x(), p.y(), p.z());
      chassisMesh.quaternion.set(q.x(), q.y(), q.z(), q.w());
    }

    syncList.push(sync);
  }

  function createObjects() {

    //createBox(new THREE.Vector3(0, -0.5, 0), ZERO_QUATERNION, 75, 1, 75, 0, 2);

    var quaternion = new THREE.Quaternion(0, 0, 0, 1);
    quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 18);
    createBox(new THREE.Vector3(0, 0.5, 0), quaternion, 8, 4, 10, 0);

    var size = .75;
    var nw = 8;
    var nh = 6;
    for (var j = 0; j < nw; j++)
      for (var i = 0; i < nh; i++)
        createBox(new THREE.Vector3(size * j - (size * (nw - 1)) / 2, size * i, 10), ZERO_QUATERNION, size, size, size, 10);

    createVehicle(new THREE.Vector3(0, 4, -20), ZERO_QUATERNION);
  }

  function getHeightData(img) {
    var c = document.createElement('canvas'),
      g = c.getContext('2d');

    c.width = img.width;
    c.height = img.height;
    g.drawImage(img, 0, 0, c.width, c.height);

    var hfdata = new Array( c.width * c.height );
    var maxHeight = 35.0;
    var minHeight = 1.0;
    var totalHeight = maxHeight - minHeight;

    let i = 0;
    for (let y = 0; y < c.height; ++y) {
      for (let x = 0; x < c.width; ++x) {
        let p = g.getImageData(y, c.width - x - 1, 1, 1).data;
        let a = (p[0] + p[1] + p[2]) / (255 + 255 + 255);
        a = a * totalHeight + minHeight;
        hfdata[i++] = a;
      }
    }

    return hfdata;
  }

  function createTerrainShape(
    terrainWidthExtents, terrainDepthExtents,
    terrainWidth, terrainDepth,
    heightData, terrainMinHeight, terrainMaxHeight) {
    // This parameter is not really used, since we are using PHY_FLOAT height data type and hence it is ignored
    var heightScale = 1;

    // Up axis = 0 for X, 1 for Y, 2 for Z. Normally 1 = Y is used.
    var upAxis = 1;

    // hdt, height data type. "PHY_FLOAT" is used. Possible values are "PHY_FLOAT", "PHY_UCHAR", "PHY_SHORT"
    var hdt = "PHY_FLOAT";

    // Set this to your needs (inverts the triangles)
    var flipQuadEdges = false;

    // Creates height data buffer in Ammo heap
    ammoHeightData = Ammo._malloc( 4 * terrainWidth * terrainDepth );

    // Copy the javascript height data array to the Ammo one.
    var p = 0;
    var p2 = 0;
    for ( var j = 0; j < terrainDepth; j ++ ) {
      for ( var i = 0; i < terrainWidth; i ++ ) {

        // write 32-bit float data to memory
        Ammo.HEAPF32[ammoHeightData + p2 >> 2] = heightData[ p ];

        p ++;

        // 4 bytes/float
        p2 += 4;
      }
    }

    // Creates the heightfield physics shape
    var heightFieldShape = new Ammo.btHeightfieldTerrainShape(

      terrainWidth,
      terrainDepth,

      ammoHeightData,

      heightScale,
      terrainMinHeight,
      terrainMaxHeight,

      upAxis,
      hdt,
      flipQuadEdges
    );

    // Set horizontal scale
    var scaleX = terrainWidthExtents / ( terrainWidth - 1 );
    var scaleZ = terrainDepthExtents / ( terrainDepth - 1 );
    heightFieldShape.setLocalScaling( new Ammo.btVector3( scaleX, 1, scaleZ ) );

    heightFieldShape.setMargin( 0.05 );

    return heightFieldShape;
  }

  function createTerrainMesh(terrainWidthExtents, terrainDepthExtents, terrainWidth, terrainDepth, heightData) {
    let geometry = new THREE.PlaneBufferGeometry( terrainWidthExtents, terrainDepthExtents, terrainWidth - 1, terrainDepth - 1 );
    geometry.rotateX( - Math.PI / 2 );

    var vertices = geometry.attributes.position.array;

    for (let i = 0, j = 0, l = vertices.length; i < l; i ++, j += 3) {
      // j + 1 because it is the y component that we modify
      vertices[ j + 1 ] = heightData[ i ];
    }

    geometry.computeVertexNormals();

    let groundTexture = new THREE.TextureLoader().load('textures/sand.jpg');
    groundTexture.wrapS = groundTexture.wrapT = THREE.RepeatWrapping;
    groundTexture.repeat.set( 150, 150 );

    let groundMaterial = new THREE.MeshLambertMaterial( { map: groundTexture, flatShading: true } );
    let terrainMesh = new THREE.Mesh( geometry, groundMaterial );
    scene.add( terrainMesh );
  }

  function createHeightmap(img) {
    let heights = getHeightData(img);
    let shape = createTerrainShape( 800, 800, img.width, img.height,
      heights, 1.0, 35.0);

    let groundTransform = new Ammo.btTransform();
    groundTransform.setIdentity();
    // Shifts the terrain, since bullet re-centers it on its bounding box.
    groundTransform.setOrigin( new Ammo.btVector3( 0, ( 35.0 + 1.0 ) / 2, 0 ) );
    var groundMass = 0;
    var groundLocalInertia = new Ammo.btVector3( 0, 0, 0 );
    var groundMotionState = new Ammo.btDefaultMotionState( groundTransform );
    var groundBody = new Ammo.btRigidBody( new Ammo.btRigidBodyConstructionInfo( groundMass, groundMotionState, shape, groundLocalInertia ) );
    physicsWorld.addRigidBody( groundBody );

    createTerrainMesh(800, 800, img.width, img.height, heights);
  }

  // - Init -
  initGraphics();
  initPhysics();
  var img = new Image();
  img.onload = function() {
    createHeightmap(this);
    createObjects();
    animate();
  };
  img.src = 'textures/heightfield.png';

});
