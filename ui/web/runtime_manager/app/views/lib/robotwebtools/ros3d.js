/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author David Gossow - dgossow@willowgarage.com
 */

var ROS3D = ROS3D || {
  REVISION : '0.16.0'
};

// Marker types
ROS3D.MARKER_ARROW = 0;
ROS3D.MARKER_CUBE = 1;
ROS3D.MARKER_SPHERE = 2;
ROS3D.MARKER_CYLINDER = 3;
ROS3D.MARKER_LINE_STRIP = 4;
ROS3D.MARKER_LINE_LIST = 5;
ROS3D.MARKER_CUBE_LIST = 6;
ROS3D.MARKER_SPHERE_LIST = 7;
ROS3D.MARKER_POINTS = 8;
ROS3D.MARKER_TEXT_VIEW_FACING = 9;
ROS3D.MARKER_MESH_RESOURCE = 10;
ROS3D.MARKER_TRIANGLE_LIST = 11;

// Interactive marker feedback types
ROS3D.INTERACTIVE_MARKER_KEEP_ALIVE = 0;
ROS3D.INTERACTIVE_MARKER_POSE_UPDATE = 1;
ROS3D.INTERACTIVE_MARKER_MENU_SELECT = 2;
ROS3D.INTERACTIVE_MARKER_BUTTON_CLICK = 3;
ROS3D.INTERACTIVE_MARKER_MOUSE_DOWN = 4;
ROS3D.INTERACTIVE_MARKER_MOUSE_UP = 5;

// Interactive marker control types
ROS3D.INTERACTIVE_MARKER_NONE = 0;
ROS3D.INTERACTIVE_MARKER_MENU = 1;
ROS3D.INTERACTIVE_MARKER_BUTTON = 2;
ROS3D.INTERACTIVE_MARKER_MOVE_AXIS = 3;
ROS3D.INTERACTIVE_MARKER_MOVE_PLANE = 4;
ROS3D.INTERACTIVE_MARKER_ROTATE_AXIS = 5;
ROS3D.INTERACTIVE_MARKER_MOVE_ROTATE = 6;

// Interactive marker rotation behavior
ROS3D.INTERACTIVE_MARKER_INHERIT = 0;
ROS3D.INTERACTIVE_MARKER_FIXED = 1;
ROS3D.INTERACTIVE_MARKER_VIEW_FACING = 2;

// Collada loader types
ROS3D.COLLADA_LOADER = 1;
ROS3D.COLLADA_LOADER_2 = 2;


/**
 * Create a THREE material based on the given RGBA values.
 *
 * @param r - the red value
 * @param g - the green value
 * @param b - the blue value
 * @param a - the alpha value
 * @returns the THREE material
 */
ROS3D.makeColorMaterial = function(r, g, b, a) {
  var color = new THREE.Color();
  color.setRGB(r, g, b);
  if (a <= 0.99) {
    return new THREE.MeshBasicMaterial({
      color : color.getHex(),
      opacity : a + 0.1,
      transparent : true,
      depthWrite : true,
      blendSrc : THREE.SrcAlphaFactor,
      blendDst : THREE.OneMinusSrcAlphaFactor,
      blendEquation : THREE.ReverseSubtractEquation,
      blending : THREE.NormalBlending
    });
  } else {
    return new THREE.MeshPhongMaterial({
      color : color.getHex(),
      opacity : a,
      blending : THREE.NormalBlending
    });
  }
};

/**
 * Return the intersection between the mouseray and the plane.
 *
 * @param mouseRay - the mouse ray
 * @param planeOrigin - the origin of the plane
 * @param planeNormal - the normal of the plane
 * @returns the intersection point
 */
ROS3D.intersectPlane = function(mouseRay, planeOrigin, planeNormal) {
  var vector = new THREE.Vector3();
  var intersectPoint = new THREE.Vector3();
  vector.subVectors(planeOrigin, mouseRay.origin);
  var dot = mouseRay.direction.dot(planeNormal);

  // bail if ray and plane are parallel
  if (Math.abs(dot) < mouseRay.precision) {
    return undefined;
  }

  // calc distance to plane
  var scalar = planeNormal.dot(vector) / dot;

  intersectPoint.addVectors(mouseRay.origin, mouseRay.direction.clone().multiplyScalar(scalar));
  return intersectPoint;
};

/**
 * Find the closest point on targetRay to any point on mouseRay. Math taken from
 * http://paulbourke.net/geometry/lineline3d/
 *
 * @param targetRay - the target ray to use
 * @param mouseRay - the mouse ray
 * @param the closest point between the two rays
 */
ROS3D.findClosestPoint = function(targetRay, mouseRay) {
  var v13 = new THREE.Vector3();
  v13.subVectors(targetRay.origin, mouseRay.origin);
  var v43 = mouseRay.direction.clone();
  var v21 = targetRay.direction.clone();
  var d1343 = v13.dot(v43);
  var d4321 = v43.dot(v21);
  var d1321 = v13.dot(v21);
  var d4343 = v43.dot(v43);
  var d2121 = v21.dot(v21);

  var denom = d2121 * d4343 - d4321 * d4321;
  // check within a delta
  if (Math.abs(denom) <= 0.0001) {
    return undefined;
  }
  var numer = d1343 * d4321 - d1321 * d4343;

  var mua = numer / denom;
  return mua;
};

/**
 * Find the closest point between the axis and the mouse.
 *
 * @param axisRay - the ray from the axis
 * @param camera - the camera to project from
 * @param mousePos - the mouse position
 * @returns the closest axis point
 */
ROS3D.closestAxisPoint = function(axisRay, camera, mousePos) {
  var projector = new THREE.Projector();

  // project axis onto screen
  var o = axisRay.origin.clone();
  projector.projectVector(o, camera);
  var o2 = axisRay.direction.clone().add(axisRay.origin);
  projector.projectVector(o2, camera);

  // d is the axis vector in screen space (d = o2-o)
  var d = o2.clone().sub(o);

  // t is the 2d ray param of perpendicular projection of mousePos onto o
  var tmp = new THREE.Vector2();
  // (t = (mousePos - o) * d / (d*d))
  var t = tmp.subVectors(mousePos, o).dot(d) / d.dot(d);

  // mp is the final 2d-projected mouse pos (mp = o + d*t)
  var mp = new THREE.Vector2();
  mp.addVectors(o, d.clone().multiplyScalar(t));

  // go back to 3d by shooting a ray
  var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
  projector.unprojectVector(vector, camera);
  var mpRay = new THREE.Ray(camera.position, vector.sub(camera.position).normalize());

  return ROS3D.findClosestPoint(axisRay, mpRay);
};

/**
 * @author Julius Kammerl - jkammerl@willowgarage.com
 */

/**
 * The DepthCloud object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * url - the URL of the stream
 *   * f (optional) - the camera's focal length (defaults to standard Kinect calibration)
 *   * pointSize (optional) - point size (pixels) for rendered point cloud
 *   * width (optional) - width of the video stream
 *   * height (optional) - height of the video stream
 *   * whiteness (optional) - blends rgb values to white (0..100)
 *   * varianceThreshold (optional) - threshold for variance filter, used for compression artifact removal
 */
ROS3D.DepthCloud = function(options) {
  options = options || {};
  THREE.Object3D.call(this);

  this.url = options.url;
  this.f = options.f || 526;
  this.pointSize = options.pointSize || 3;
  this.width = options.width || 1024;
  this.height = options.height || 1024;
  this.whiteness = options.whiteness || 0;
  this.varianceThreshold = options.varianceThreshold || 0.000016667;

  var metaLoaded = false;
  this.video = document.createElement('video');

  this.video.addEventListener('loadedmetadata', this.metaLoaded.bind(this), false);

  this.video.loop = true;
  this.video.src = this.url;
  this.video.crossOrigin = 'Anonymous';
  this.video.setAttribute('crossorigin', 'Anonymous');

  // define custom shaders
  this.vertex_shader = [
    'uniform sampler2D map;',
    '',
    'uniform float width;',
    'uniform float height;',
    'uniform float nearClipping, farClipping;',
    '',
    'uniform float pointSize;',
    'uniform float zOffset;',
    '',
    'uniform float focallength;',
    '',
    'varying vec2 vUvP;',
    'varying vec2 colorP;',
    '',
    'varying float depthVariance;',
    'varying float maskVal;',
    '',
    'float sampleDepth(vec2 pos)',
    '  {',
    '    float depth;',
    '    ',
    '    vec2 vUv = vec2( pos.x / (width*2.0), pos.y / (height*2.0)+0.5 );',
    '    vec2 vUv2 = vec2( pos.x / (width*2.0)+0.5, pos.y / (height*2.0)+0.5 );',
    '    ',
    '    vec4 depthColor = texture2D( map, vUv );',
    '    ',
    '    depth = ( depthColor.r + depthColor.g + depthColor.b ) / 3.0 ;',
    '    ',
    '    if (depth>0.99)',
    '    {',
    '      vec4 depthColor2 = texture2D( map, vUv2 );',
    '      float depth2 = ( depthColor2.r + depthColor2.g + depthColor2.b ) / 3.0 ;',
    '      depth = 0.99+depth2;',
    '    }',
    '    ',
    '    return depth;',
    '  }',
    '',
    'float median(float a, float b, float c)',
    '  {',
    '    float r=a;',
    '    ',
    '    if ( (a<b) && (b<c) )',
    '    {',
    '      r = b;',
    '    }',
    '    if ( (a<c) && (c<b) )',
    '    {',
    '      r = c;',
    '    }',
    '    return r;',
    '  }',
    '',
    'float variance(float d1, float d2, float d3, float d4, float d5, float d6, float d7, float d8, float d9)',
    '  {',
    '    float mean = (d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8 + d9) / 9.0;',
    '    float t1 = (d1-mean);',
    '    float t2 = (d2-mean);',
    '    float t3 = (d3-mean);',
    '    float t4 = (d4-mean);',
    '    float t5 = (d5-mean);',
    '    float t6 = (d6-mean);',
    '    float t7 = (d7-mean);',
    '    float t8 = (d8-mean);',
    '    float t9 = (d9-mean);',
    '    float v = (t1*t1+t2*t2+t3*t3+t4*t4+t5*t5+t6*t6+t7*t7+t8*t8+t9*t9)/9.0;',
    '    return v;',
    '  }',
    '',
    'vec2 decodeDepth(vec2 pos)',
    '  {',
    '    vec2 ret;',
    '    ',
    '    ',
    '    float depth1 = sampleDepth(vec2(position.x-1.0, position.y-1.0));',
    '    float depth2 = sampleDepth(vec2(position.x, position.y-1.0));',
    '    float depth3 = sampleDepth(vec2(position.x+1.0, position.y-1.0));',
    '    float depth4 = sampleDepth(vec2(position.x-1.0, position.y));',
    '    float depth5 = sampleDepth(vec2(position.x, position.y));',
    '    float depth6 = sampleDepth(vec2(position.x+1.0, position.y));',
    '    float depth7 = sampleDepth(vec2(position.x-1.0, position.y+1.0));',
    '    float depth8 = sampleDepth(vec2(position.x, position.y+1.0));',
    '    float depth9 = sampleDepth(vec2(position.x+1.0, position.y+1.0));',
    '    ',
    '    float median1 = median(depth1, depth2, depth3);',
    '    float median2 = median(depth4, depth5, depth6);',
    '    float median3 = median(depth7, depth8, depth9);',
    '    ',
    '    ret.x = median(median1, median2, median3);',
    '    ret.y = variance(depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9);',
    '    ',
    '    return ret;',
    '    ',
    '  }',
    '',
    '',
    'void main() {',
    '  ',
    '  vUvP = vec2( position.x / (width*2.0), position.y / (height*2.0)+0.5 );',
    '  colorP = vec2( position.x / (width*2.0)+0.5 , position.y / (height*2.0)  );',
    '  ',
    '  vec4 pos = vec4(0.0,0.0,0.0,0.0);',
    '  depthVariance = 0.0;',
    '  ',
    '  if ( (vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>0.0))',
    '  {',
    '    vec2 smp = decodeDepth(vec2(position.x, position.y));',
    '    float depth = smp.x;',
    '    depthVariance = smp.y;',
    '    ',
    '    float z = -depth;',
    '    ',
    '    pos = vec4(',
    '      ( position.x / width - 0.5 ) * z * (1000.0/focallength) * -1.0,',
    '      ( position.y / height - 0.5 ) * z * (1000.0/focallength),',
    '      (- z + zOffset / 1000.0) * 2.0,',
    '      1.0);',
    '    ',
    '    vec2 maskP = vec2( position.x / (width*2.0), position.y / (height*2.0)  );',
    '    vec4 maskColor = texture2D( map, maskP );',
    '    maskVal = ( maskColor.r + maskColor.g + maskColor.b ) / 3.0 ;',
    '  }',
    '  ',
    '  gl_PointSize = pointSize;',
    '  gl_Position = projectionMatrix * modelViewMatrix * pos;',
    '  ',
    '}'
    ].join('\n');

  this.fragment_shader = [
    'uniform sampler2D map;',
    'uniform float varianceThreshold;',
    'uniform float whiteness;',
    '',
    'varying vec2 vUvP;',
    'varying vec2 colorP;',
    '',
    'varying float depthVariance;',
    'varying float maskVal;',
    '',
    '',
    'void main() {',
    '  ',
    '  vec4 color;',
    '  ',
    '  if ( (depthVariance>varianceThreshold) || (maskVal>0.5) ||(vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>1.0))',
    '  {  ',
    '    discard;',
    '  }',
    '  else ',
    '  {',
    '    color = texture2D( map, colorP );',
    '    ',
    '    float fader = whiteness /100.0;',
    '    ',
    '    color.r = color.r * (1.0-fader)+ fader;',
    '    ',
    '    color.g = color.g * (1.0-fader)+ fader;',
    '    ',
    '    color.b = color.b * (1.0-fader)+ fader;',
    '    ',
    '    color.a = 1.0;//smoothstep( 20000.0, -20000.0, gl_FragCoord.z / gl_FragCoord.w );',
    '  }',
    '  ',
    '  gl_FragColor = vec4( color.r, color.g, color.b, color.a );',
    '  ',
    '}'
    ].join('\n');
};
ROS3D.DepthCloud.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Callback called when video metadata is ready
 */
ROS3D.DepthCloud.prototype.metaLoaded = function() {
  this.metaLoaded = true;
  this.initStreamer();
};

/**
 * Callback called when video metadata is ready
 */
ROS3D.DepthCloud.prototype.initStreamer = function() {

  if (this.metaLoaded) {
    this.texture = new THREE.Texture(this.video);
    this.geometry = new THREE.Geometry();

    for (var i = 0, l = this.width * this.height; i < l; i++) {

      var vertex = new THREE.Vector3();
      vertex.x = (i % this.width);
      vertex.y = Math.floor(i / this.width);

      this.geometry.vertices.push(vertex);
    }

    this.material = new THREE.ShaderMaterial({
      uniforms : {
        'map' : {
          type : 't',
          value : this.texture
        },
        'width' : {
          type : 'f',
          value : this.width
        },
        'height' : {
          type : 'f',
          value : this.height
        },
        'focallength' : {
          type : 'f',
          value : this.f
        },
        'pointSize' : {
          type : 'f',
          value : this.pointSize
        },
        'zOffset' : {
          type : 'f',
          value : 0
        },
        'whiteness' : {
          type : 'f',
          value : this.whiteness
        },
        'varianceThreshold' : {
          type : 'f',
          value : this.varianceThreshold
        }
      },
      vertexShader : this.vertex_shader,
      fragmentShader : this.fragment_shader
    });

    this.mesh = new THREE.ParticleSystem(this.geometry, this.material);
    this.mesh.position.x = 0;
    this.mesh.position.y = 0;
    this.add(this.mesh);

    var that = this;

    setInterval(function() {
      if (that.video.readyState === that.video.HAVE_ENOUGH_DATA) {
        that.texture.needsUpdate = true;
      }
    }, 1000 / 30);
  }
};

/**
 * Start video playback
 */
ROS3D.DepthCloud.prototype.startStream = function() {
  this.video.play();
};

/**
 * Stop video playback
 */
ROS3D.DepthCloud.prototype.stopStream = function() {
  this.video.pause();
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main interactive marker object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * handle - the ROS3D.InteractiveMarkerHandle for this marker
 *  * camera - the main camera associated with the viewer for this marker
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                        ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.InteractiveMarker = function(options) {
  THREE.Object3D.call(this);
  THREE.EventDispatcher.call(this);

  var that = this;
  options = options || {};
  var handle = options.handle;
  this.name = handle.name;
  var camera = options.camera;
  var path = options.path || '/';
  var loader = options.loader || ROS3D.COLLADA_LOADER_2;
  this.dragging = false;

  // set the initial pose
  this.onServerSetPose({
    pose : handle.pose
  });

  // information on where the drag started
  this.dragStart = {
    position : new THREE.Vector3(),
    orientation : new THREE.Quaternion(),
    positionWorld : new THREE.Vector3(),
    orientationWorld : new THREE.Quaternion(),
    event3d : {}
  };

  // add each control message
  handle.controls.forEach(function(controlMessage) {
    that.add(new ROS3D.InteractiveMarkerControl({
      parent : that,
      handle : handle,
      message : controlMessage,
      camera : camera,
      path : path,
      loader : loader
    }));
  });

  // check for any menus
  if (handle.menuEntries.length > 0) {
    this.menu = new ROS3D.InteractiveMarkerMenu({
      menuEntries : handle.menuEntries,
      menuFontSize : handle.menuFontSize
    });

    // forward menu select events
    this.menu.addEventListener('menu-select', function(event) {
      that.dispatchEvent(event);
    });
  }
};
ROS3D.InteractiveMarker.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Show the interactive marker menu associated with this marker.
 *
 * @param control - the control to use
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.showMenu = function(control, event) {
  if (this.menu) {
    this.menu.show(control, event);
  }
};

/**
 * Move the axis based on the given event information.
 *
 * @param control - the control to use
 * @param origAxis - the origin of the axis
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.moveAxis = function(control, origAxis, event3d) {
  if (this.dragging) {
    var currentControlOri = control.currentControlOri;
    var axis = origAxis.clone().applyQuaternion(currentControlOri);
    // get move axis in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var axisWorld = axis.clone().applyQuaternion(this.dragStart.orientationWorld.clone());

    var axisRay = new THREE.Ray(originWorld, axisWorld);

    // find closest point to mouse on axis
    var t = ROS3D.closestAxisPoint(axisRay, event3d.camera, event3d.mousePos);

    // offset from drag start position
    var p = new THREE.Vector3();
    p.addVectors(this.dragStart.position, axis.clone().applyQuaternion(this.dragStart.orientation)
        .multiplyScalar(t));
    this.setPosition(control, p);

    event3d.stopPropagation();
  }
};

/**
 * Move with respect to the plane based on the contorl and event.
 *
 * @param control - the control to use
 * @param origNormal - the normal of the origin
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.movePlane = function(control, origNormal, event3d) {
  if (this.dragging) {
    var currentControlOri = control.currentControlOri;
    var normal = origNormal.clone().applyQuaternion(currentControlOri);
    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = normal.clone().applyQuaternion(this.dragStart.orientationWorld);

    // intersect mouse ray with plane
    var intersection = ROS3D.intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset from drag start position
    var p = new THREE.Vector3();
    p.subVectors(intersection, originWorld);
    p.add(this.dragStart.positionWorld);
    this.setPosition(control, p);
    event3d.stopPropagation();
  }
};

/**
 * Rotate based on the control and event given.
 *
 * @param control - the control to use
 * @param origOrientation - the orientation of the origin
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.rotateAxis = function(control, origOrientation, event3d) {
  if (this.dragging) {
    control.updateMatrixWorld();

    var currentControlOri = control.currentControlOri;
    var orientation = currentControlOri.clone().multiply(origOrientation.clone());

    var normal = (new THREE.Vector3(1, 0, 0)).applyQuaternion(orientation);

    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = normal.applyQuaternion(this.dragStart.orientationWorld);

    // intersect mouse ray with plane
    var intersection = ROS3D.intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset local origin to lie on intersection plane
    var normalRay = new THREE.Ray(this.dragStart.positionWorld, normalWorld);
    var rotOrigin = ROS3D.intersectPlane(normalRay, originWorld, normalWorld);

    // rotates from world to plane coords
    var orientationWorld = this.dragStart.orientationWorld.clone().multiply(orientation);
    var orientationWorldInv = orientationWorld.clone().inverse();

    // rotate original and current intersection into local coords
    intersection.sub(rotOrigin);
    intersection.applyQuaternion(orientationWorldInv);

    var origIntersection = this.dragStart.event3d.intersection.point.clone();
    origIntersection.sub(rotOrigin);
    origIntersection.applyQuaternion(orientationWorldInv);

    // compute relative 2d angle
    var a1 = Math.atan2(intersection.y, intersection.z);
    var a2 = Math.atan2(origIntersection.y, origIntersection.z);
    var a = a2 - a1;

    var rot = new THREE.Quaternion();
    rot.setFromAxisAngle(normal, a);

    // rotate
    this.setOrientation(control, rot.multiply(this.dragStart.orientationWorld));

    // offset from drag start position
    event3d.stopPropagation();
  }
};

/**
 * Dispatch the given event type.
 *
 * @param type - the type of event
 * @param control - the control to use
 */
ROS3D.InteractiveMarker.prototype.feedbackEvent = function(type, control) {
  this.dispatchEvent({
    type : type,
    position : this.position.clone(),
    orientation : this.quaternion.clone(),
    controlName : control.name
  });
};

/**
 * Start a drag action.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.startDrag = function(control, event3d) {
  if (event3d.domEvent.button === 0) {
    event3d.stopPropagation();
    this.dragging = true;
    this.updateMatrixWorld(true);
    var scale = new THREE.Vector3();
    this.matrixWorld
        .decompose(this.dragStart.positionWorld, this.dragStart.orientationWorld, scale);
    this.dragStart.position = this.position.clone();
    this.dragStart.orientation = this.quaternion.clone();
    this.dragStart.event3d = event3d;

    this.feedbackEvent('user-mousedown', control);
  }
};

/**
 * Stop a drag action.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.stopDrag = function(control, event3d) {
  if (event3d.domEvent.button === 0) {
    event3d.stopPropagation();
    this.dragging = false;
    this.dragStart.event3d = {};
    this.onServerSetPose(this.bufferedPoseEvent);
    this.bufferedPoseEvent = undefined;

    this.feedbackEvent('user-mouseup', control);
  }
};

/**
 * Handle a button click.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.buttonClick = function(control, event3d) {
  event3d.stopPropagation();
  this.feedbackEvent('user-button-click', control);
};

/**
 * Handle a user pose change for the position.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.setPosition = function(control, position) {
  this.position = position;
  this.feedbackEvent('user-pose-change', control);
};

/**
 * Handle a user pose change for the orientation.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.setOrientation = function(control, orientation) {
  orientation.normalize();
  this.quaternion = orientation;
  this.feedbackEvent('user-pose-change', control);
};

/**
 * Update the marker based when the pose is set from the server.
 *
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.onServerSetPose = function(event) {
  if (event !== undefined) {
    // don't update while dragging
    if (this.dragging) {
      this.bufferedPoseEvent = event;
    } else {
      var pose = event.pose;

      this.position.x = pose.position.x;
      this.position.y = pose.position.y;
      this.position.z = pose.position.z;

      this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y,
          pose.orientation.z, pose.orientation.w);

      this.updateMatrixWorld(true);
    }
  }
};

/**
 * Free memory of elements in this marker.
 */
ROS3D.InteractiveMarker.prototype.dispose = function() {
  var that = this;
  this.children.forEach(function(intMarkerControl) {
    intMarkerControl.children.forEach(function(marker) {
      marker.dispose();
      intMarkerControl.remove(marker);
    });
    that.remove(intMarkerControl);
  });
};

THREE.EventDispatcher.prototype.apply( ROS3D.InteractiveMarker.prototype );

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A client for an interactive marker topic.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - a handle to the ROS connection
 *  * tfClient - a handle to the TF client
 *  * topic (optional) - the topic to subscribe to, like '/basic_controls'
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * camera - the main camera associated with the viewer for this marker client
 *  * rootObject (optional) - the root THREE 3D object to render to
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                        ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 *  * menuFontSize (optional) - the menu font size
 */
ROS3D.InteractiveMarkerClient = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topic = options.topic;
  this.path = options.path || '/';
  this.camera = options.camera;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;
  this.menuFontSize = options.menuFontSize || '0.8em';

  this.interactiveMarkers = {};
  this.updateTopic = null;
  this.feedbackTopic = null;

  // check for an initial topic
  if (this.topic) {
    this.subscribe(this.topic);
  }
};

/**
 * Subscribe to the given interactive marker topic. This will unsubscribe from any current topics.
 *
 * @param topic - the topic to subscribe to, like '/basic_controls'
 */
ROS3D.InteractiveMarkerClient.prototype.subscribe = function(topic) {
  // unsubscribe to the other topics
  this.unsubscribe();

  this.updateTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : topic + '/tunneled/update',
    messageType : 'visualization_msgs/InteractiveMarkerUpdate',
    compression : 'png'
  });
  this.updateTopic.subscribe(this.processUpdate.bind(this));

  this.feedbackTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : topic + '/feedback',
    messageType : 'visualization_msgs/InteractiveMarkerFeedback',
    compression : 'png'
  });
  this.feedbackTopic.advertise();

  this.initService = new ROSLIB.Service({
    ros : this.ros,
    name : topic + '/tunneled/get_init',
    serviceType : 'demo_interactive_markers/GetInit'
  });
  var request = new ROSLIB.ServiceRequest({});
  this.initService.callService(request, this.processInit.bind(this));
};

/**
 * Unsubscribe from the current interactive marker topic.
 */
ROS3D.InteractiveMarkerClient.prototype.unsubscribe = function() {
  if (this.updateTopic) {
    this.updateTopic.unsubscribe();
  }
  if (this.feedbackTopic) {
    this.feedbackTopic.unadvertise();
  }
  // erase all markers
  for (var intMarkerName in this.interactiveMarkers) {
    this.eraseIntMarker(intMarkerName);
  }
  this.interactiveMarkers = {};
};

/**
 * Process the given interactive marker initialization message.
 *
 * @param initMessage - the interactive marker initialization message to process
 */
ROS3D.InteractiveMarkerClient.prototype.processInit = function(initMessage) {
  var message = initMessage.msg;

  // erase any old markers
  message.erases = [];
  for (var intMarkerName in this.interactiveMarkers) {
    message.erases.push(intMarkerName);
  }
  message.poses = [];

  // treat it as an update
  this.processUpdate(message);
};

/**
 * Process the given interactive marker update message.
 *
 * @param initMessage - the interactive marker update message to process
 */
ROS3D.InteractiveMarkerClient.prototype.processUpdate = function(message) {
  var that = this;

  // erase any markers
  message.erases.forEach(function(name) {
    that.eraseIntMarker(name);
  });

  // updates marker poses
  message.poses.forEach(function(poseMessage) {
    var marker = that.interactiveMarkers[poseMessage.name];
    if (marker) {
      marker.setPoseFromServer(poseMessage.pose);
    }
  });

  // add new markers
  message.markers.forEach(function(msg) {
    // get rid of anything with the same name
    var oldhandle = that.interactiveMarkers[msg.name];
    if (oldhandle) {
      that.eraseIntMarker(oldhandle.name);
    }

    // create the handle
    var handle = new ROS3D.InteractiveMarkerHandle({
      message : msg,
      feedbackTopic : that.feedbackTopic,
      tfClient : that.tfClient,
      menuFontSize : that.menuFontSize
    });
    that.interactiveMarkers[msg.name] = handle;

    // create the actual marker
    var intMarker = new ROS3D.InteractiveMarker({
      handle : handle,
      camera : that.camera,
      path : that.path,
      loader : that.loader
    });
    // add it to the scene
    intMarker.name = msg.name;
    that.rootObject.add(intMarker);

    // listen for any pose updates from the server
    handle.on('pose', function(pose) {
      intMarker.onServerSetPose({
        pose : pose
      });
    });

    intMarker.addEventListener('user-pose-change', handle.setPoseFromClient.bind(handle));
    intMarker.addEventListener('user-mousedown', handle.onMouseDown.bind(handle));
    intMarker.addEventListener('user-mouseup', handle.onMouseUp.bind(handle));
    intMarker.addEventListener('user-button-click', handle.onButtonClick.bind(handle));
    intMarker.addEventListener('menu-select', handle.onMenuSelect.bind(handle));

    // now list for any TF changes
    handle.subscribeTf();
  });
};

/**
 * Erase the interactive marker with the given name.
 *
 * @param intMarkerName - the interactive marker name to delete
 */
ROS3D.InteractiveMarkerClient.prototype.eraseIntMarker = function(intMarkerName) {
  if (this.interactiveMarkers[intMarkerName]) {
    // remove the object
    var targetIntMarker = this.rootObject.getObjectByName(intMarkerName);
    this.rootObject.remove(targetIntMarker);
    delete this.interactiveMarkers[intMarkerName];
    targetIntMarker.dispose();
  }
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main marker control object for an interactive marker.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * parent - the parent of this control
 *  * message - the interactive marker control message
 *  * camera - the main camera associated with the viewer for this marker client
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                        ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.InteractiveMarkerControl = function(options) {
  var that = this;
  THREE.Object3D.call(this);

  options = options || {};
  this.parent = options.parent;
  var handle = options.handle;
  var message = options.message;
  this.name = message.name;
  this.camera = options.camera;
  this.path = options.path || '/';
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;
  this.dragging = false;
  this.startMousePos = new THREE.Vector2();

  // orientation for the control
  var controlOri = new THREE.Quaternion(message.orientation.x, message.orientation.y,
      message.orientation.z, message.orientation.w);
  controlOri.normalize();

  // transform x axis into local frame
  var controlAxis = new THREE.Vector3(1, 0, 0);
  controlAxis.applyQuaternion(controlOri);

  this.currentControlOri = new THREE.Quaternion();

  // determine mouse interaction
  switch (message.interaction_mode) {
    case ROS3D.INTERACTIVE_MARKER_MOVE_AXIS:
      this.addEventListener('mousemove', this.parent.moveAxis.bind(this.parent, this, controlAxis));
      this.addEventListener('touchmove', this.parent.moveAxis.bind(this.parent, this, controlAxis));
      break;
    case ROS3D.INTERACTIVE_MARKER_ROTATE_AXIS:
      this
          .addEventListener('mousemove', this.parent.rotateAxis.bind(this.parent, this, controlOri));
      break;
    case ROS3D.INTERACTIVE_MARKER_MOVE_PLANE:
      this
          .addEventListener('mousemove', this.parent.movePlane.bind(this.parent, this, controlAxis));
      break;
    case ROS3D.INTERACTIVE_MARKER_BUTTON:
      this.addEventListener('click', this.parent.buttonClick.bind(this.parent, this));
      break;
    default:
      break;
  }

  /**
   * Install default listeners for highlighting / dragging.
   *
   * @param event - the event to stop
   */
  function stopPropagation(event) {
    event.stopPropagation();
  }

  // check the mode
  if (message.interaction_mode !== ROS3D.INTERACTIVE_MARKER_NONE) {
    this.addEventListener('mousedown', this.parent.startDrag.bind(this.parent, this));
    this.addEventListener('mouseup', this.parent.stopDrag.bind(this.parent, this));
    this.addEventListener('contextmenu', this.parent.showMenu.bind(this.parent, this));
    this.addEventListener('mouseup', function(event3d) {
      if (that.startMousePos.distanceToSquared(event3d.mousePos) === 0) {
        event3d.type = 'contextmenu';
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('mouseover', stopPropagation);
    this.addEventListener('mouseout', stopPropagation);
    this.addEventListener('click', stopPropagation);
    this.addEventListener('mousedown', function(event3d) {
      that.startMousePos = event3d.mousePos;
    });

    // touch support
    this.addEventListener('touchstart', function(event3d) {
      if (event3d.domEvent.touches.length === 1) {
        event3d.type = 'mousedown';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchmove', function(event3d) {
      if (event3d.domEvent.touches.length === 1) {
        event3d.type = 'mousemove';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchend', function(event3d) {
      if (event3d.domEvent.touches.length === 0) {
        event3d.domEvent.button = 0;
        event3d.type = 'mouseup';
        that.dispatchEvent(event3d);
        event3d.type = 'click';
        that.dispatchEvent(event3d);
      }
    });
  }

  // rotation behavior
  var rotInv = new THREE.Quaternion();
  var posInv = this.parent.position.clone().multiplyScalar(-1);
  switch (message.orientation_mode) {
    case ROS3D.INTERACTIVE_MARKER_INHERIT:
      rotInv = this.parent.quaternion.clone().inverse();
      this.updateMatrixWorld = function(force) {
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
        that.currentControlOri.normalize();
      };
      break;
    case ROS3D.INTERACTIVE_MARKER_FIXED:
      this.updateMatrixWorld = function(force) {
        that.quaternion = that.parent.quaternion.clone().inverse();
        that.updateMatrix();
        that.matrixWorldNeedsUpdate = true;
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
      };
      break;
    case ROS3D.INTERACTIVE_MARKER_VIEW_FACING:
      var independentMarkerOrientation = message.independent_marker_orientation;
      this.updateMatrixWorld = function(force) {
        that.camera.updateMatrixWorld();
        var cameraRot = new THREE.Matrix4().extractRotation(that.camera.matrixWorld);

        var ros2Gl = new THREE.Matrix4();
        var r90 = Math.PI * 0.5;
        var rv = new THREE.Euler(-r90, 0, r90);
        ros2Gl.makeRotationFromEuler(rv);

        var worldToLocal = new THREE.Matrix4();
        worldToLocal.getInverse(that.parent.matrixWorld);

        cameraRot.multiplyMatrices(cameraRot, ros2Gl);
        cameraRot.multiplyMatrices(worldToLocal, cameraRot);

        that.currentControlOri.setFromRotationMatrix(cameraRot);

        // check the orientation
        if (!independentMarkerOrientation) {
          that.quaternion.copy(that.currentControlOri);
          that.updateMatrix();
          that.matrixWorldNeedsUpdate = true;
        }
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      };
      break;
    default:
      console.error('Unkown orientation mode: ' + message.orientation_mode);
      break;
  }

  // temporary TFClient to get transformations from InteractiveMarker
  // frame to potential child Marker frames
  var localTfClient = new ROSLIB.TFClient({
    ros : handle.tfClient.ros,
    fixedFrame : handle.message.header.frame_id,
  });

  // create visuals (markers)
  message.markers.forEach(function(markerMsg) {
    var addMarker = function(transformMsg) {
      var markerHelper = new ROS3D.Marker({
        message : markerMsg,
        path : that.path,
        loader : that.loader
      });
      
      // if transformMsg isn't null, this was called by TFClient
      if (transformMsg !== null) {
        // get the current pose as a ROSLIB.Pose...
        var newPose = new ROSLIB.Pose({
          position : markerHelper.position,
          orientation : markerHelper.quaternion
        });
        // so we can apply the transform provided by the TFClient
        newPose.applyTransform(new ROSLIB.Transform(transformMsg));
        markerHelper.setPose(newPose);

        markerHelper.updateMatrixWorld();
        // we only need to set the pose once - at least, this is what RViz seems to be doing, might change in the future
        localTfClient.unsubscribe(markerMsg.header.frame_id);
      }

      // add the marker
      that.add(markerHelper);
    };
    
    // If the marker lives in a separate TF Frame, ask the *local* TFClient
    // for the transformation from the InteractiveMarker frame to the
    // sub-Marker frame
    if (markerMsg.header.frame_id !== '') {
      localTfClient.subscribe(markerMsg.header.frame_id, addMarker);
    }
    // If not, just add the marker without changing its pose
    else {
      addMarker(null);
    }
  });
};
ROS3D.InteractiveMarkerControl.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * Handle with signals for a single interactive marker.
 *
 * Emits the following events:
 *
 *  * 'pose' - emitted when a new pose comes from the server
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * message - the interactive marker message
 *  * feedbackTopic - the ROSLIB.Topic associated with the feedback
 *  * tfClient - a handle to the TF client to use
 *  * menuFontSize (optional) - the menu font size
 */
ROS3D.InteractiveMarkerHandle = function(options) {
  options = options || {};
  this.message = options.message;
  this.feedbackTopic = options.feedbackTopic;
  this.tfClient = options.tfClient;
  this.menuFontSize = options.menuFontSize || '0.8em';
  this.name = this.message.name;
  this.header = this.message.header;
  this.controls = this.message.controls;
  this.menuEntries = this.message.menu_entries;
  this.dragging = false;
  this.timeoutHandle = null;
  this.tfTransform = new ROSLIB.Transform();
  this.pose = new ROSLIB.Pose();

  // start by setting the pose
  this.setPoseFromServer(this.message.pose);
};
ROS3D.InteractiveMarkerHandle.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Subscribe to the TF associated with this interactive marker.
 */
ROS3D.InteractiveMarkerHandle.prototype.subscribeTf = function() {
  // subscribe to tf updates if frame-fixed
  if (this.message.header.stamp.secs === 0.0 && this.message.header.stamp.nsecs === 0.0) {
    this.tfClient.subscribe(this.message.header.frame_id, this.tfUpdate.bind(this));
  }
};

/**
 * Emit the new pose that has come from the server.
 */
ROS3D.InteractiveMarkerHandle.prototype.emitServerPoseUpdate = function() {
  var poseTransformed = new ROSLIB.Pose(this.pose);
  poseTransformed.applyTransform(this.tfTransform);
  this.emit('pose', poseTransformed);
};

/**
 * Update the pose based on the pose given by the server.
 *
 * @param poseMsg - the pose given by the server
 */
ROS3D.InteractiveMarkerHandle.prototype.setPoseFromServer = function(poseMsg) {
  this.pose = new ROSLIB.Pose(poseMsg);
  this.emitServerPoseUpdate();
};

/**
 * Update the pose based on the TF given by the server.
 *
 * @param transformMsg - the TF given by the server
 */
ROS3D.InteractiveMarkerHandle.prototype.tfUpdate = function(transformMsg) {
  this.tfTransform = new ROSLIB.Transform(transformMsg);
  this.emitServerPoseUpdate();
};

/**
 * Set the pose from the client based on the given event.
 *
 * @param event - the event to base the change off of
 */
ROS3D.InteractiveMarkerHandle.prototype.setPoseFromClient = function(event) {
  // apply the transform
  this.pose = new ROSLIB.Pose(event);
  var inv = this.tfTransform.clone();
  inv.rotation.invert();
  inv.translation.multiplyQuaternion(inv.rotation);
  inv.translation.x *= -1;
  inv.translation.y *= -1;
  inv.translation.z *= -1;
  this.pose.applyTransform(inv);

  // send feedback to the server
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_POSE_UPDATE, undefined, 0, event.controlName);

  // keep sending pose feedback until the mouse goes up
  if (this.dragging) {
    if (this.timeoutHandle) {
      clearTimeout(this.timeoutHandle);
    }
    this.timeoutHandle = setTimeout(this.setPoseFromClient.bind(this, event), 250);
  }
};

/**
 * Send the button click feedback to the server.
 *
 * @param event - the event associated with the button click
 */
ROS3D.InteractiveMarkerHandle.prototype.onButtonClick = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_BUTTON_CLICK, event.clickPosition, 0,
      event.controlName);
};

/**
 * Send the mousedown feedback to the server.
 *
 * @param event - the event associated with the mousedown
 */
ROS3D.InteractiveMarkerHandle.prototype.onMouseDown = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MOUSE_DOWN, event.clickPosition, 0, event.controlName);
  this.dragging = true;
};

/**
 * Send the mouseup feedback to the server.
 *
 * @param event - the event associated with the mouseup
 */
ROS3D.InteractiveMarkerHandle.prototype.onMouseUp = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MOUSE_UP, event.clickPosition, 0, event.controlName);
  this.dragging = false;
  if (this.timeoutHandle) {
    clearTimeout(this.timeoutHandle);
  }
};

/**
 * Send the menu select feedback to the server.
 *
 * @param event - the event associated with the menu select
 */
ROS3D.InteractiveMarkerHandle.prototype.onMenuSelect = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MENU_SELECT, undefined, event.id, event.controlName);
};

/**
 * Send feedback to the interactive marker server.
 *
 * @param eventType - the type of event that happened
 * @param clickPosition (optional) - the position in ROS space the click happened
 * @param menuEntryID (optional) - the menu entry ID that is associated
 * @param controlName - the name of the control
 */
ROS3D.InteractiveMarkerHandle.prototype.sendFeedback = function(eventType, clickPosition,
    menuEntryID, controlName) {

  // check for the click position
  var mousePointValid = clickPosition !== undefined;
  clickPosition = clickPosition || {
    x : 0,
    y : 0,
    z : 0
  };

  var feedback = {
    header : this.header,
    client_id : this.clientID,
    marker_name : this.name,
    control_name : controlName,
    event_type : eventType,
    pose : this.pose,
    mouse_point : clickPosition,
    mouse_point_valid : mousePointValid,
    menu_entry_id : menuEntryID
  };
  this.feedbackTopic.publish(feedback);
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A menu for an interactive marker. This will be overlayed on the canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * menuEntries - the menu entries to add
 *  * className (optional) - a custom CSS class for the menu div
 *  * entryClassName (optional) - a custom CSS class for the menu entry
 *  * overlayClassName (optional) - a custom CSS class for the menu overlay
 *  * menuFontSize (optional) - the menu font size
 */
ROS3D.InteractiveMarkerMenu = function(options) {
  var that = this;
  options = options || {};
  var menuEntries = options.menuEntries;
  var className = options.className || 'default-interactive-marker-menu';
  var entryClassName = options.entryClassName || 'default-interactive-marker-menu-entry';
  var overlayClassName = options.overlayClassName || 'default-interactive-marker-overlay';
  var menuFontSize = options.menuFontSize || '0.8em';

  // holds the menu tree
  var allMenus = [];
  allMenus[0] = {
    children : []
  };

  THREE.EventDispatcher.call(this);

  // create the CSS for this marker if it has not been created
  if (document.getElementById('default-interactive-marker-menu-css') === null) {
    var style = document.createElement('style');
    style.id = 'default-interactive-marker-menu-css';
    style.type = 'text/css';
    style.innerHTML = '.default-interactive-marker-menu {' + 'background-color: #444444;'
        + 'border: 1px solid #888888;' + 'border: 1px solid #888888;' + 'padding: 0px 0px 0px 0px;'
        + 'color: #FFFFFF;' + 'font-family: sans-serif;' + 'font-size: ' + menuFontSize +';' + 'z-index: 1002;'
        + '}' + '.default-interactive-marker-menu ul {' + 'padding: 0px 0px 5px 0px;'
        + 'margin: 0px;' + 'list-style-type: none;' + '}'
        + '.default-interactive-marker-menu ul li div {' + '-webkit-touch-callout: none;'
        + '-webkit-user-select: none;' + '-khtml-user-select: none;' + '-moz-user-select: none;'
        + '-ms-user-select: none;' + 'user-select: none;' + 'cursor: default;'
        + 'padding: 3px 10px 3px 10px;' + '}' + '.default-interactive-marker-menu-entry:hover {'
        + '  background-color: #666666;' + '  cursor: pointer;' + '}'
        + '.default-interactive-marker-menu ul ul {' + '  font-style: italic;'
        + '  padding-left: 10px;' + '}' + '.default-interactive-marker-overlay {'
        + '  position: absolute;' + '  top: 0%;' + '  left: 0%;' + '  width: 100%;'
        + '  height: 100%;' + '  background-color: black;' + '  z-index: 1001;'
        + '  -moz-opacity: 0.0;' + '  opacity: .0;' + '  filter: alpha(opacity = 0);' + '}';
    document.getElementsByTagName('head')[0].appendChild(style);
  }

  // place the menu in a div
  this.menuDomElem = document.createElement('div');
  this.menuDomElem.style.position = 'absolute';
  this.menuDomElem.className = className;
  this.menuDomElem.addEventListener('contextmenu', function(event) {
    event.preventDefault();
  });

  // create the overlay DOM
  this.overlayDomElem = document.createElement('div');
  this.overlayDomElem.className = overlayClassName;

  this.hideListener = this.hide.bind(this);
  this.overlayDomElem.addEventListener('contextmenu', this.hideListener);
  this.overlayDomElem.addEventListener('click', this.hideListener);
  this.overlayDomElem.addEventListener('touchstart', this.hideListener);

  // parse all entries and link children to parents
  var i, entry, id;
  for ( i = 0; i < menuEntries.length; i++) {
    entry = menuEntries[i];
    id = entry.id;
    allMenus[id] = {
      title : entry.title,
      id : id,
      children : []
    };
  }
  for ( i = 0; i < menuEntries.length; i++) {
    entry = menuEntries[i];
    id = entry.id;
    var menu = allMenus[id];
    var parent = allMenus[entry.parent_id];
    parent.children.push(menu);
  }

  function emitMenuSelect(menuEntry, domEvent) {
    this.dispatchEvent({
      type : 'menu-select',
      domEvent : domEvent,
      id : menuEntry.id,
      controlName : this.controlName
    });
    this.hide(domEvent);
  }

  /**
   * Create the HTML UL element for the menu and link it to the parent.
   *
   * @param parentDomElem - the parent DOM element
   * @param parentMenu - the parent menu
   */
  function makeUl(parentDomElem, parentMenu) {

    var ulElem = document.createElement('ul');
    parentDomElem.appendChild(ulElem);

    var children = parentMenu.children;

    for ( var i = 0; i < children.length; i++) {
      var liElem = document.createElement('li');
      var divElem = document.createElement('div');
      divElem.appendChild(document.createTextNode(children[i].title));
      ulElem.appendChild(liElem);
      liElem.appendChild(divElem);

      if (children[i].children.length > 0) {
        makeUl(liElem, children[i]);
        divElem.addEventListener('click', that.hide.bind(that));
        divElem.addEventListener('touchstart', that.hide.bind(that));
      } else {
        divElem.addEventListener('click', emitMenuSelect.bind(that, children[i]));
        divElem.addEventListener('touchstart', emitMenuSelect.bind(that, children[i]));
        divElem.className = 'default-interactive-marker-menu-entry';
      }
    }

  }

  // construct DOM element
  makeUl(this.menuDomElem, allMenus[0]);
};

/**
 * Shoe the menu DOM element.
 *
 * @param control - the control for the menu
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarkerMenu.prototype.show = function(control, event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  this.controlName = control.name;

  // position it on the click
  if (event.domEvent.changedTouches !== undefined) {
    // touch click
    this.menuDomElem.style.left = event.domEvent.changedTouches[0].pageX + 'px';
    this.menuDomElem.style.top = event.domEvent.changedTouches[0].pageY + 'px';
  } else {
    // mouse click
    this.menuDomElem.style.left = event.domEvent.clientX + 'px';
    this.menuDomElem.style.top = event.domEvent.clientY + 'px';
  }
  document.body.appendChild(this.overlayDomElem);
  document.body.appendChild(this.menuDomElem);
};

/**
 * Hide the menu DOM element.
 *
 * @param event (optional) - the event that caused this
 */
ROS3D.InteractiveMarkerMenu.prototype.hide = function(event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  document.body.removeChild(this.overlayDomElem);
  document.body.removeChild(this.menuDomElem);
};

THREE.EventDispatcher.prototype.apply( ROS3D.InteractiveMarkerMenu.prototype );

/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A Marker can convert a ROS marker message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * path - the base path or URL for any mesh files that will be loaded for this marker
 *   * message - the marker message
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.Marker = function(options) {
  options = options || {};
  var path = options.path || '/';
  var message = options.message;
  var loader = options.loader || ROS3D.COLLADA_LOADER_2;

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    path += '/';
  }

  THREE.Object3D.call(this);
  
  if(message.scale) {
    this.msgScale = [message.scale.x, message.scale.y, message.scale.z];
  }
  else {
    this.msgScale = [1,1,1];
  }
  this.msgColor = message.color;
  this.msgMesh = undefined;

  // set the pose and get the color
  this.setPose(message.pose);
  var colorMaterial = ROS3D.makeColorMaterial(this.msgColor.r,
      this.msgColor.g, this.msgColor.b, this.msgColor.a);

  // create the object based on the type
  switch (message.type) {
    case ROS3D.MARKER_ARROW:
      // get the sizes for the arrow
      var len = message.scale.x;
      var headLength = len * 0.23;
      var headDiameter = message.scale.y;
      var shaftDiameter = headDiameter * 0.5;

      // determine the points
      var direction, p1 = null;
      if (message.points.length === 2) {
        p1 = new THREE.Vector3(message.points[0].x, message.points[0].y, message.points[0].z);
        var p2 = new THREE.Vector3(message.points[1].x, message.points[1].y, message.points[1].z);
        direction = p1.clone().negate().add(p2);
        // direction = p2 - p1;
        len = direction.length();
        headDiameter = message.scale.y;
        shaftDiameter = message.scale.x;

        if (message.scale.z !== 0.0) {
          headLength = message.scale.z;
        }
      }

      // add the marker
      this.add(new ROS3D.Arrow({
        direction : direction,
        origin : p1,
        length : len,
        headLength : headLength,
        shaftDiameter : shaftDiameter,
        headDiameter : headDiameter,
        material : colorMaterial
      }));
      break;
    case ROS3D.MARKER_CUBE:
      // set the cube dimensions
      var cubeGeom = new THREE.CubeGeometry(message.scale.x, message.scale.y, message.scale.z);
      this.add(new THREE.Mesh(cubeGeom, colorMaterial));
      break;
    case ROS3D.MARKER_SPHERE:
      // set the sphere dimensions
      var sphereGeom = new THREE.SphereGeometry(0.5);
      var sphereMesh = new THREE.Mesh(sphereGeom, colorMaterial);
      sphereMesh.scale.x = message.scale.x;
      sphereMesh.scale.y = message.scale.y;
      sphereMesh.scale.z = message.scale.z;
      this.add(sphereMesh);
      break;
    case ROS3D.MARKER_CYLINDER:
      // set the cylinder dimensions
      var cylinderGeom = new THREE.CylinderGeometry(0.5, 0.5, 1, 16, 1, false);
      var cylinderMesh = new THREE.Mesh(cylinderGeom, colorMaterial);
      cylinderMesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
      cylinderMesh.scale = new THREE.Vector3(message.scale.x, message.scale.z, message.scale.y);
      this.add(cylinderMesh);
      break;
    case ROS3D.MARKER_LINE_STRIP:
      var lineStripGeom = new THREE.Geometry();
      var lineStripMaterial = new THREE.LineBasicMaterial({
        size : message.scale.x
      });

      // add the points
      var j;
      for ( j = 0; j < message.points.length; j++) {
        var pt = new THREE.Vector3();
        pt.x = message.points[j].x;
        pt.y = message.points[j].y;
        pt.z = message.points[j].z;
        lineStripGeom.vertices.push(pt);
      }

      // determine the colors for each
      if (message.colors.length === message.points.length) {
        lineStripMaterial.vertexColors = true;
        for ( j = 0; j < message.points.length; j++) {
          var clr = new THREE.Color();
          clr.setRGB(message.colors[j].r, message.colors[j].g, message.colors[j].b);
          lineStripGeom.colors.push(clr);
        }
      } else {
        lineStripMaterial.color.setRGB(message.color.r, message.color.g, message.color.b);
      }

      // add the line
      this.add(new THREE.Line(lineStripGeom, lineStripMaterial));
      break;
    case ROS3D.MARKER_LINE_LIST:
      var lineListGeom = new THREE.Geometry();
      var lineListMaterial = new THREE.LineBasicMaterial({
        size : message.scale.x
      });

      // add the points
      var k;
      for ( k = 0; k < message.points.length; k++) {
        var v = new THREE.Vector3();
        v.x = message.points[k].x;
        v.y = message.points[k].y;
        v.z = message.points[k].z;
        lineListGeom.vertices.push(v);
      }

      // determine the colors for each
      if (message.colors.length === message.points.length) {
        lineListMaterial.vertexColors = true;
        for ( k = 0; k < message.points.length; k++) {
          var c = new THREE.Color();
          c.setRGB(message.colors[k].r, message.colors[k].g, message.colors[k].b);
          lineListGeom.colors.push(c);
        }
      } else {
        lineListMaterial.color.setRGB(message.color.r, message.color.g, message.color.b);
      }

      // add the line
      this.add(new THREE.Line(lineListGeom, lineListMaterial,THREE.LinePieces));
      break;
    case ROS3D.MARKER_CUBE_LIST:
      // holds the main object
      var object = new THREE.Object3D();
      
      // check if custom colors should be used
      var numPoints = message.points.length;
      var createColors = (numPoints === message.colors.length);
      // do not render giant lists
      var stepSize = Math.ceil(numPoints / 1250);
        
      // add the points
      var p, cube, curColor, newMesh;
      for (p = 0; p < numPoints; p+=stepSize) {
        cube = new THREE.CubeGeometry(message.scale.x, message.scale.y, message.scale.z);

        // check the color
        if(createColors) {
          curColor = ROS3D.makeColorMaterial(message.colors[p].r, message.colors[p].g, message.colors[p].b, message.colors[p].a);
        } else {
          curColor = colorMaterial;
        }

        newMesh = new THREE.Mesh(cube, curColor);
        newMesh.position.x = message.points[p].x;
        newMesh.position.y = message.points[p].y;
        newMesh.position.z = message.points[p].z;
        object.add(newMesh);
      }

      this.add(object);
      break;
    case ROS3D.MARKER_SPHERE_LIST:
    case ROS3D.MARKER_POINTS:
      // for now, use a particle system for the lists
      var geometry = new THREE.Geometry();
      var material = new THREE.ParticleBasicMaterial({
        size : message.scale.x
      });

      // add the points
      var i;
      for ( i = 0; i < message.points.length; i++) {
        var vertex = new THREE.Vector3();
        vertex.x = message.points[i].x;
        vertex.y = message.points[i].y;
        vertex.z = message.points[i].z;
        geometry.vertices.push(vertex);
      }

      // determine the colors for each
      if (message.colors.length === message.points.length) {
        material.vertexColors = true;
        for ( i = 0; i < message.points.length; i++) {
          var color = new THREE.Color();
          color.setRGB(message.colors[i].r, message.colors[i].g, message.colors[i].b);
          geometry.colors.push(color);
        }
      } else {
        material.color.setRGB(message.color.r, message.color.g, message.color.b);
      }

      // add the particle system
      this.add(new THREE.ParticleSystem(geometry, material));
      break;
    case ROS3D.MARKER_TEXT_VIEW_FACING:
      // only work on non-empty text
      if (message.text.length > 0) {
        // Use a THREE.Sprite to always be view-facing
        // ( code from http://stackoverflow.com/a/27348780 )
        var textColor = this.msgColor;

        var canvas = document.createElement('canvas');
        var context = canvas.getContext('2d');
        var textHeight = 100;
        var fontString = 'normal ' + textHeight + 'px sans-serif';
        context.font = fontString;
        var metrics = context.measureText( message.text );
        var textWidth = metrics.width;

        canvas.width = textWidth;
        canvas.height = textHeight;

        // this does need to be set again
        context.font = fontString;
        context.fillStyle = 'rgba('
          + 255*textColor.r + ', '
          + 255*textColor.g + ', '
          + 255*textColor.b + ', '
          + textColor.a + ')';
        context.textAlign = 'left';
        context.textBaseline = 'middle';
        context.fillText( message.text, 0, canvas.height/2);

        var texture = new THREE.Texture(canvas);
        texture.needsUpdate = true;

        var spriteMaterial = new THREE.SpriteMaterial({
          map: texture,
          // NOTE: This is needed for THREE.js r61, unused in r70
          useScreenCoordinates: false });
        var sprite = new THREE.Sprite( spriteMaterial );
        var textSize = message.scale.z;
        sprite.scale.set(textWidth / canvas.height * textSize, textSize, 1);

        this.add(sprite);      }
      break;
    case ROS3D.MARKER_MESH_RESOURCE:
      // load and add the mesh
      var meshColorMaterial = null;
      if(message.color.r !== 0 || message.color.g !== 0 ||
         message.color.b !== 0 || message.color.a !== 0) {
        meshColorMaterial = colorMaterial;
      }
      this.msgMesh = message.mesh_resource.substr(10);
      var meshResource = new ROS3D.MeshResource({
        path : path,
        resource :  this.msgMesh,
        material : meshColorMaterial,
        loader : loader
      });
      this.add(meshResource);
      break;
    case ROS3D.MARKER_TRIANGLE_LIST:
      // create the list of triangles
      var tri = new ROS3D.TriangleList({
        material : colorMaterial,
        vertices : message.points,
        colors : message.colors
      });
      tri.scale = new THREE.Vector3(message.scale.x, message.scale.y, message.scale.z);
      this.add(tri);
      break;
    default:
      console.error('Currently unsupported marker type: ' + message.type);
      break;
  }
};
ROS3D.Marker.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the pose of this marker to the given values.
 *
 * @param pose - the pose to set for this marker
 */
ROS3D.Marker.prototype.setPose = function(pose) {
  // set position information
  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  // set the rotation
  this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  this.quaternion.normalize();

  // update the world
  this.updateMatrixWorld();
};

/**
 * Update this marker.
 *
 * @param message - the marker message
 * @return true on success otherwhise false is returned
 */
ROS3D.Marker.prototype.update = function(message) {
  // set the pose and get the color
  this.setPose(message.pose);
  
  // Update color
  if(message.color.r !== this.msgColor.r ||
     message.color.g !== this.msgColor.g ||
     message.color.b !== this.msgColor.b ||
     message.color.a !== this.msgColor.a)
  {
      var colorMaterial = ROS3D.makeColorMaterial(
          message.color.r, message.color.g,
          message.color.b, message.color.a);
  
      switch (message.type) {
      case ROS3D.MARKER_LINE_STRIP:
      case ROS3D.MARKER_LINE_LIST:
      case ROS3D.MARKER_POINTS:
          break;
      case ROS3D.MARKER_ARROW:
      case ROS3D.MARKER_CUBE:
      case ROS3D.MARKER_SPHERE:
      case ROS3D.MARKER_CYLINDER:
      case ROS3D.MARKER_TRIANGLE_LIST:
      case ROS3D.MARKER_TEXT_VIEW_FACING:
          this.traverse (function (child){
              if (child instanceof THREE.Mesh) {
                  child.material = colorMaterial;
              }
          });
          break;
      case ROS3D.MARKER_MESH_RESOURCE:
          var meshColorMaterial = null;
          if(message.color.r !== 0 || message.color.g !== 0 ||
             message.color.b !== 0 || message.color.a !== 0) {
              meshColorMaterial = this.colorMaterial;
          }
          this.traverse (function (child){
              if (child instanceof THREE.Mesh) {
                  child.material = meshColorMaterial;
              }
          });
          break;
      case ROS3D.MARKER_CUBE_LIST:
      case ROS3D.MARKER_SPHERE_LIST:
          // TODO Support to update color for MARKER_CUBE_LIST & MARKER_SPHERE_LIST
          return false;
      default:
          return false;
      }
      
      this.msgColor = message.color;
  }
  
  // Update geometry
  var scaleChanged =
        Math.abs(this.msgScale[0] - message.scale.x) > 1.0e-6 ||
        Math.abs(this.msgScale[1] - message.scale.y) > 1.0e-6 ||
        Math.abs(this.msgScale[2] - message.scale.z) > 1.0e-6;
  this.msgScale = [message.scale.x, message.scale.y, message.scale.z];
  
  switch (message.type) {
    case ROS3D.MARKER_CUBE:
    case ROS3D.MARKER_SPHERE:
    case ROS3D.MARKER_CYLINDER:
        if(scaleChanged) {
            return false;
        }
        break;
    case ROS3D.MARKER_TEXT_VIEW_FACING:
        if(scaleChanged || this.text !== message.text) {
            return false;
        }
        break;
    case ROS3D.MARKER_MESH_RESOURCE:
        var meshResource = message.mesh_resource.substr(10);
        if(meshResource !== this.msgMesh) {
            return false;
        }
        if(scaleChanged) {
            return false;
        }
        break;
    case ROS3D.MARKER_ARROW:
    case ROS3D.MARKER_LINE_STRIP:
    case ROS3D.MARKER_LINE_LIST:
    case ROS3D.MARKER_CUBE_LIST:
    case ROS3D.MARKER_SPHERE_LIST:
    case ROS3D.MARKER_POINTS:
    case ROS3D.MARKER_TRIANGLE_LIST:
        // TODO: Check if geometry changed
        return false;
    default:
        break;
  }
  
  return true;
};

/*
 * Free memory of elements in this marker.
 */
ROS3D.Marker.prototype.dispose = function() {
  this.children.forEach(function(element) {
    if (element instanceof ROS3D.MeshResource) {
      element.children.forEach(function(scene) {
        if (scene.material !== undefined) {
          scene.material.dispose();
        }
        scene.children.forEach(function(mesh) {
          if (mesh.geometry !== undefined) {
            mesh.geometry.dispose();
          }
          if (mesh.material !== undefined) {
            mesh.material.dispose();
          }
          scene.remove(mesh);
        });
        element.remove(scene);
      });
    } else {
      if (element.geometry !== undefined) {
          element.geometry.dispose();
      }
      if (element.material !== undefined) {
          element.material.dispose();
      }
    }
    element.parent.remove(element);
  });
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Nils Berg - berg.nils@gmail.com
 */

/**
 * A MarkerArray client that listens to a given topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the MarkerArray
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * rootObject (optional) - the root object to add the markers to
 *   * path (optional) - the base path to any meshes that will be loaded
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MarkerArrayClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};

  // subscribe to MarkerArray topic
  var arrayTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/MarkerArray',
    compression : 'png'
  });
  
  arrayTopic.subscribe(function(arrayMessage) {
    arrayMessage.markers.forEach(function(message) {
      if(message.action === 0) {
        var updated = false;
        if(message.ns + message.id in that.markers) { // "MODIFY"
          updated = that.markers[message.ns + message.id].children[0].update(message);
          if(!updated) { // "REMOVE"
              that.rootObject.remove(that.markers[message.ns + message.id]);
          }
        }
        if(!updated) { // "ADD"
          var newMarker = new ROS3D.Marker({
            message : message,
            path : that.path,
            loader : that.loader
          });
          that.markers[message.ns + message.id] = new ROS3D.SceneNode({
            frameID : message.header.frame_id,
            tfClient : that.tfClient,
            object : newMarker
          });
          that.rootObject.add(that.markers[message.ns + message.id]);
        }
      }
      else if(message.action === 1) { // "DEPRECATED"
        console.warn('Received marker message with deprecated action identifier "1"');
      }
      else if(message.action === 2) { // "DELETE"
        that.rootObject.remove(that.markers[message.ns + message.id]);
        delete that.markers[message.ns + message.id];
      }
      else if(message.action === 3) { // "DELETE ALL"
        for (var m in that.markers){
          that.rootObject.remove(m);
        }
        that.markers = {};
      }
      else {
        console.warn('Received marker message with unknown action identifier "'+message.action+'"');
      }
    });
    
    that.emit('change');
  });
};
ROS3D.MarkerArrayClient.prototype.__proto__ = EventEmitter2.prototype;

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A marker client that listens to a given marker topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * rootObject (optional) - the root object to add this marker to
 *   * path (optional) - the base path to any meshes that will be loaded
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MarkerClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/Marker',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    var newMarker = new ROS3D.Marker({
      message : message,
      path : that.path,
      loader : that.loader
    });

    // remove old marker from Three.Object3D children buffer
    that.rootObject.remove(that.markers[message.ns + message.id]);

    that.markers[message.ns + message.id] = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : that.tfClient,
      object : newMarker
    });
    that.rootObject.add(that.markers[message.ns + message.id]);

    that.emit('change');
    return true;
  });
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A Arrow is a THREE object that can be used to display an arrow model.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * origin (optional) - the origin of the arrow
 *   * direction (optional) - the direction vector of the arrow
 *   * length (optional) - the length of the arrow
 *   * headLength (optional) - the head length of the arrow
 *   * shaftDiameter (optional) - the shaft diameter of the arrow
 *   * headDiameter (optional) - the head diameter of the arrow
 *   * material (optional) - the material to use for this arrow
 */
ROS3D.Arrow = function(options) {
  options = options || {};
  var origin = options.origin || new THREE.Vector3(0, 0, 0);
  var direction = options.direction || new THREE.Vector3(1, 0, 0);
  var length = options.length || 1;
  var headLength = options.headLength || 0.2;
  var shaftDiameter = options.shaftDiameter || 0.05;
  var headDiameter = options.headDiameter || 0.1;
  var material = options.material || new THREE.MeshBasicMaterial();

  var shaftLength = length - headLength;

  // create and merge geometry
  var geometry = new THREE.CylinderGeometry(shaftDiameter * 0.5, shaftDiameter * 0.5, shaftLength,
      12, 1);
  var m = new THREE.Matrix4();
  m.setPosition(new THREE.Vector3(0, shaftLength * 0.5, 0));
  geometry.applyMatrix(m);

  // create the head
  var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5, headLength, 12, 1);
  m.setPosition(new THREE.Vector3(0, shaftLength + (headLength * 0.5), 0));
  coneGeometry.applyMatrix(m);

  // put the arrow together
  THREE.GeometryUtils.merge(geometry, coneGeometry);

  THREE.Mesh.call(this, geometry, material);

  this.position = origin;
  this.setDirection(direction);
};
ROS3D.Arrow.prototype.__proto__ = THREE.Mesh.prototype;

/**
 * Set the direction of this arrow to that of the given vector.
 *
 * @param direction - the direction to set this arrow
 */
ROS3D.Arrow.prototype.setDirection = function(direction) {
  var axis = new THREE.Vector3(0, 1, 0).cross(direction);
  var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(direction.clone().normalize()));
  this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);
  this.rotation.setFromRotationMatrix(this.matrix, this.rotation.order);
};

/**
 * Set this arrow to be the given length.
 *
 * @param length - the new length of the arrow
 */
ROS3D.Arrow.prototype.setLength = function(length) {
  this.scale.set(length, length, length);
};

/**
 * Set the color of this arrow to the given hex value.
 *
 * @param hex - the hex value of the color to use
 */
ROS3D.Arrow.prototype.setColor = function(hex) {
  this.geometry.material.color.setHex(hex);
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * An Axes object can be used to display the axis of a particular coordinate frame.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * shaftRadius (optional) - the radius of the shaft to render
 *   * headRadius (optional) - the radius of the head to render
 *   * headLength (optional) - the length of the head to render
 */
ROS3D.Axes = function(options) {
  var that = this;
  options = options || {};
  var shaftRadius = options.shaftRadius || 0.008;
  var headRadius = options.headRadius || 0.023;
  var headLength = options.headLength || 0.1;

  THREE.Object3D.call(this);

  // create the cylinders for the objects
  this.lineGeom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, 1.0 - headLength);
  this.headGeom = new THREE.CylinderGeometry(0, headRadius, headLength);

  /**
   * Adds an axis marker to this axes object.
   *
   * @param axis - the 3D vector representing the axis to add
   */
  function addAxis(axis) {
    // set the color of the axis
    var color = new THREE.Color();
    color.setRGB(axis.x, axis.y, axis.z);
    var material = new THREE.MeshBasicMaterial({
      color : color.getHex()
    });

    // setup the rotation information
    var rotAxis = new THREE.Vector3();
    rotAxis.crossVectors(axis, new THREE.Vector3(0, -1, 0));
    var rot = new THREE.Quaternion();
    rot.setFromAxisAngle(rotAxis, 0.5 * Math.PI);

    // create the arrow
    var arrow = new THREE.Mesh(that.headGeom, material);
    arrow.position = axis.clone();
    arrow.position.multiplyScalar(0.95);
    arrow.quaternion = rot;
    arrow.updateMatrix();
    that.add(arrow);

    // create the line
    var line = new THREE.Mesh(that.lineGeom, material);
    line.position = axis.clone();
    line.position.multiplyScalar(0.45);
    line.quaternion = rot;
    line.updateMatrix();
    that.add(line);
  }

  // add the three markers to the axes
  addAxis(new THREE.Vector3(1, 0, 0));
  addAxis(new THREE.Vector3(0, 1, 0));
  addAxis(new THREE.Vector3(0, 0, 1));
};
ROS3D.Axes.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Create a grid object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * num_cells (optional) - The number of cells of the grid
 *  * color (optional) - the line color of the grid, like '#cccccc'
 *  * lineWidth (optional) - the width of the lines in the grid
 *  * cellSize (optional) - The length, in meters, of the side of each cell
 */
ROS3D.Grid = function(options) {
  options = options || {};
  var num_cells = options.num_cells || 10;
  var color = options.color || '#cccccc';
  var lineWidth = options.lineWidth || 1;
  var cellSize = options.cellSize || 1;

  THREE.Object3D.call(this);

  var material = new THREE.LineBasicMaterial({
    color: color,
    linewidth: lineWidth
  });

  for (var i = 0; i <= num_cells; ++i) {
    var edge = cellSize * num_cells / 2;
    var position = edge - (i * cellSize);
    var geometryH = new THREE.Geometry();
    geometryH.vertices.push(
      new THREE.Vector3( -edge, position, 0 ),
      new THREE.Vector3( edge, position, 0 )
    );
    var geometryV = new THREE.Geometry();
    geometryV.vertices.push(
      new THREE.Vector3( position, -edge, 0 ),
      new THREE.Vector3( position, edge, 0 )
    );
    this.add(new THREE.Line(geometryH, material));
    this.add(new THREE.Line(geometryV, material));
  }
};

ROS3D.Grid.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A MeshResource is an THREE object that will load from a external mesh file. Currently loads
 * Collada files.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * path (optional) - the base path to the associated models that will be loaded
 *  * resource - the resource file name to load
 *  * material (optional) - the material to use for the object
 *  * warnings (optional) - if warnings should be printed
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                        ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MeshResource = function(options) {
  var that = this;
  options = options || {};
  var path = options.path || '/';
  var resource = options.resource;
  var material = options.material || null;
  this.warnings = options.warnings;
  var loaderType = options.loader || ROS3D.COLLADA_LOADER_2;

  THREE.Object3D.call(this);

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    this.path += '/';
  }

  var uri = path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  var loader;
  if (fileType === '.dae') {
    if (loaderType ===  ROS3D.COLLADA_LOADER) {
      loader = new THREE.ColladaLoader();
    } else {
      loader = new ColladaLoader2();
    }
    loader.log = function(message) {
      if (that.warnings) {
        console.warn(message);
      }
    };
    loader.load(uri, function colladaReady(collada) {
      // check for a scale factor in ColladaLoader2
      if(loaderType === ROS3D.COLLADA_LOADER_2 && collada.dae.asset.unit) {
        var scale = collada.dae.asset.unit;
        collada.scene.scale = new THREE.Vector3(scale, scale, scale);
      }

      // add a texture to anything that is missing one
      if(material !== null) {
        var setMaterial = function(node, material) {
          // do not overwrite the material
          if (typeof node.material === 'undefined') {
            node.material = material;
          }
          //node.material = material;
          if (node.children) {
            for (var i = 0; i < node.children.length; i++) {
              setMaterial(node.children[i], material);
            }
          }
        };

        setMaterial(collada.scene, material);
      }

      that.add(collada.scene);
    });
  } else if (fileType === '.stl') {
    loader = new THREE.STLLoader();
    {
      loader.load(uri, function ( geometry ) {
        geometry.computeFaceNormals();
        var mesh;
        if(material !== null) {
          mesh = new THREE.Mesh( geometry, material );
        } else {
          mesh = new THREE.Mesh( geometry, new THREE.MeshBasicMaterial( { color: 0x999999 } ) );
        }
        that.add(mesh);
      } );
    }
  }
};
ROS3D.MeshResource.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A TriangleList is a THREE object that can be used to display a list of triangles as a geometry.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * material (optional) - the material to use for the object
 *   * vertices - the array of vertices to use
 *   * colors - the associated array of colors to use
 */
ROS3D.TriangleList = function(options) {
  options = options || {};
  var material = options.material || new THREE.MeshBasicMaterial();
  var vertices = options.vertices;
  var colors = options.colors;

  THREE.Object3D.call(this);

  // set the material to be double sided
  material.side = THREE.DoubleSide;

  // construct the geometry
  var geometry = new THREE.Geometry();
  for (i = 0; i < vertices.length; i++) {
    geometry.vertices.push(new THREE.Vector3(vertices[i].x, vertices[i].y, vertices[i].z));
  }

  // set the colors
  var i, j;
  if (colors.length === vertices.length) {
    // use per-vertex color
    for (i = 0; i < vertices.length; i += 3) {
      var faceVert = new THREE.Face3(i, i + 1, i + 2);
      for (j = i * 3; j < i * 3 + 3; i++) {
        var color = new THREE.Color();
        color.setRGB(colors[i].r, colors[i].g, colors[i].b);
        faceVert.vertexColors.push(color);
      }
      geometry.faces.push(face);
    }
    material.vertexColors = THREE.VertexColors;
  } else if (colors.length === vertices.length / 3) {
    // use per-triangle color
    for (i = 0; i < vertices.length; i += 3) {
      var faceTri = new THREE.Face3(i, i + 1, i + 2);
      faceTri.color.setRGB(colors[i / 3].r, colors[i / 3].g, colors[i / 3].b);
      geometry.faces.push(faceTri);
    }
    material.vertexColors = THREE.FaceColors;
  } else {
    // use marker color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      geometry.faces.push(face);
    }
  }

  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  geometry.computeCentroids();
  geometry.computeFaceNormals();

  this.add(new THREE.Mesh(geometry, material));
};
ROS3D.TriangleList.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the color of this object to the given hex value.
 *
 * @param hex - the hex value of the color to set
 */
ROS3D.TriangleList.prototype.setColor = function(hex) {
  this.mesh.material.color.setHex(hex);
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An OccupancyGrid can convert a ROS occupancy grid message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * message - the occupancy grid message
 */
ROS3D.OccupancyGrid = function(options) {
  options = options || {};
  var message = options.message;

  // create the geometry
  var width = message.info.width;
  var height = message.info.height;
  var geom = new THREE.PlaneGeometry(width, height);

  // internal drawing canvas
  var canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  var context = canvas.getContext('2d');
  // create the color material
  var imageData = context.createImageData(width, height);
  for ( var row = 0; row < height; row++) {
    for ( var col = 0; col < width; col++) {
      // determine the index into the map data
      var mapI = col + ((height - row - 1) * width);
      // determine the value
      var data = message.data[mapI];
      var val;
      if (data === 100) {
        val = 0;
      } else if (data === 0) {
        val = 255;
      } else {
        val = 127;
      }

      // determine the index into the image data array
      var i = (col + (row * width)) * 4;
      // r
      imageData.data[i] = val;
      // g
      imageData.data[++i] = val;
      // b
      imageData.data[++i] = val;
      // a
      imageData.data[++i] = 255;
    }
  }
  context.putImageData(imageData, 0, 0);

  var texture = new THREE.Texture(canvas);
  texture.needsUpdate = true;
  var material = new THREE.MeshBasicMaterial({
    map : texture
  });
  material.side = THREE.DoubleSide;

  // create the mesh
  THREE.Mesh.call(this, geom, material);
  // move the map so the corner is at X, Y and correct orientation (informations from message.info)
  this.useQuaternion = true;
  this.quaternion = message.info.origin.orientation;
  this.position.x = (width * message.info.resolution) / 2 + message.info.origin.position.x;
  this.position.y = (height * message.info.resolution) / 2 + message.info.origin.position.y;
  this.position.z = message.info.origin.position.z;
  this.scale.x = message.info.resolution;
  this.scale.y = message.info.resolution;
};
ROS3D.OccupancyGrid.prototype.__proto__ = THREE.Mesh.prototype;

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An occupancy grid client that listens to a given map topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * tfClient (optional) - the TF client handle to use for a scene node
 *   * rootObject (optional) - the root object to add this marker to
 */
ROS3D.OccupancyGridClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/map';
  this.continuous = options.continuous;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();

  // current grid that is displayed
  this.currentGrid = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'nav_msgs/OccupancyGrid',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    // check for an old map
    if (that.currentGrid) {
      that.rootObject.remove(that.currentGrid);
    }

    var newGrid = new ROS3D.OccupancyGrid({
      message : message
    });

    // check if we care about the scene
    if (that.tfClient) {
      that.currentGrid = new ROS3D.SceneNode({
        frameID : message.header.frame_id,
        tfClient : that.tfClient,
        object : newGrid
      });
    } else {
      that.currentGrid = newGrid;
    }

    that.rootObject.add(that.currentGrid);

    that.emit('change');

    // check if we should unsubscribe
    if (!that.continuous) {
      rosTopic.unsubscribe();
    }
  });
};
ROS3D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * An Odometry client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * keep (optional) - number of markers to keep around (default: 1)
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * length (optional) - the length of the arrow (default: 1.0)
 *  * headLength (optional) - the head length of the arrow (default: 0.2)
 *  * shaftDiameter (optional) - the shaft diameter of the arrow (default: 0.05)
 *  * headDiameter (optional) - the head diameter of the arrow (default: 0.1)
 */
ROS3D.Odometry = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/particlecloud';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.length = options.length || 1.0;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.keep = options.keep || 1;
  var that = this;
  THREE.Object3D.call(this);

  this.sns = [];

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'nav_msgs/Odometry'
  });

  rosTopic.subscribe(function(message) {
      if(that.sns.length >= that.keep) {
          that.rootObject.remove(that.sns[0]);
          that.sns.shift();
      }

      that.options.origin = new THREE.Vector3( message.pose.pose.position.x, message.pose.pose.position.y,
                                               message.pose.pose.position.z);

      var rot = new THREE.Quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                     message.pose.pose.orientation.z, message.pose.pose.orientation.w);
      that.options.direction = new THREE.Vector3(1,0,0);
      that.options.direction.applyQuaternion(rot);
      that.options.material = new THREE.MeshBasicMaterial({color: that.color});
      var arrow = new ROS3D.Arrow(that.options);

      that.sns.push(new ROS3D.SceneNode({
            frameID : message.header.frame_id,
            tfClient : that.tfClient,
            object : arrow
      }));

      that.rootObject.add(that.sns[ that.sns.length - 1]);
  });
};
ROS3D.Odometry.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A Path client that listens to a given topic and displays a line connecting the poses.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 */
ROS3D.Path = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/path';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;
  this.line = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'nav_msgs/Path'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var lineGeometry = new THREE.Geometry();
      for(var i=0; i<message.poses.length;i++){
          var v3 = new THREE.Vector3( message.poses[i].pose.position.x, message.poses[i].pose.position.y,
                                      message.poses[i].pose.position.z);
          lineGeometry.vertices.push(v3);
      }

      lineGeometry.computeLineDistances();
      var lineMaterial = new THREE.LineBasicMaterial( { color: that.color } );
      var line = new THREE.Line( lineGeometry, lineMaterial );

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : line
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Path.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PointStamped client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * radius (optional) - radius of the point (default: 0.2)
 */
ROS3D.Point = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/point';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.radius = options.radius || 0.2;
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PointStamped'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var sphereGeometry = new THREE.SphereGeometry( that.radius );
      var sphereMaterial = new THREE.MeshBasicMaterial( {color: that.color} );
      var sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
      sphere.position.set(message.point.x, message.point.y, message.point.z);

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : sphere
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Point.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PolygonStamped client that listens to a given topic and displays the polygon
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 */
ROS3D.Polygon = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/path';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;
  this.line = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PolygonStamped'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var lineGeometry = new THREE.Geometry();
      var v3;
      for(var i=0; i<message.polygon.points.length;i++){
          v3 = new THREE.Vector3( message.polygon.points[i].x, message.polygon.points[i].y,
                                  message.polygon.points[i].z);
          lineGeometry.vertices.push(v3);
      }
      v3 = new THREE.Vector3( message.polygon.points[0].x, message.polygon.points[0].y,
                              message.polygon.points[0].z);
      lineGeometry.vertices.push(v3);
      lineGeometry.computeLineDistances();
      var lineMaterial = new THREE.LineBasicMaterial( { color: that.color } );
      var line = new THREE.Line( lineGeometry, lineMaterial );

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : line
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Polygon.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PoseStamped client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * length (optional) - the length of the arrow (default: 1.0)
 *  * headLength (optional) - the head length of the arrow (default: 0.2)
 *  * shaftDiameter (optional) - the shaft diameter of the arrow (default: 0.05)
 *  * headDiameter (optional) - the head diameter of the arrow (default: 0.1)
 */
ROS3D.Pose = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/pose';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PoseStamped'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      that.options.origin = new THREE.Vector3( message.pose.position.x, message.pose.position.y,
                                               message.pose.position.z);

      var rot = new THREE.Quaternion(message.pose.orientation.x, message.pose.orientation.y,
                                     message.pose.orientation.z, message.pose.orientation.w);
      that.options.direction = new THREE.Vector3(1,0,0);
      that.options.direction.applyQuaternion(rot);
      that.options.material = new THREE.MeshBasicMaterial({color: that.color});
      var arrow = new ROS3D.Arrow(that.options);

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : arrow
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Pose.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PoseArray client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * length (optional) - the length of the arrow (default: 1.0)
 */
ROS3D.PoseArray = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/particlecloud';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.length = options.length || 1.0;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PoseArray'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var group = new THREE.Object3D();
      var line;

      for(var i=0;i<message.poses.length;i++){
          var lineGeometry = new THREE.Geometry();

          var v3 = new THREE.Vector3( message.poses[i].position.x, message.poses[i].position.y,
                                      message.poses[i].position.z);
          lineGeometry.vertices.push(v3);

          var rot = new THREE.Quaternion(message.poses[i].orientation.x, message.poses[i].orientation.y,
                                         message.poses[i].orientation.z, message.poses[i].orientation.w);

          var tip = new THREE.Vector3(that.length,0,0);
          var side1 = new THREE.Vector3(that.length*0.8, that.length*0.2, 0);
          var side2 = new THREE.Vector3(that.length*0.8, -that.length*0.2, 0);
          tip.applyQuaternion(rot);
          side1.applyQuaternion(rot);
          side2.applyQuaternion(rot);

          lineGeometry.vertices.push(tip.add(v3));
          lineGeometry.vertices.push(side1.add(v3));
          lineGeometry.vertices.push(side2.add(v3));
          lineGeometry.vertices.push(tip);

          lineGeometry.computeLineDistances();
          var lineMaterial = new THREE.LineBasicMaterial( { color: that.color } );
          line = new THREE.Line( lineGeometry, lineMaterial );

          group.add(line);
      }

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : group
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.PoseArray.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A LaserScan client that listens to a given topic and displays the points.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * color - (optional) color of the points (default 0xFFA500)
 *  * texture - (optional) Image url for a texture to use for the points. Defaults to a single white pixel.
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.LaserScan = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/scan';
  this.color = options.color || 0xFFA500;
  var that = this;

  this.particles = new ROS3D.Particles(options);

  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'sensor_msgs/LaserScan'
  });


  rosTopic.subscribe(function(message) {
    setFrame(that.particles, message.header.frame_id);

    var n = message.ranges.length;
    for(var i=0;i<n;i++){
      var range = message.ranges[i];
      if(range < message.range_min || range > message.range_max){
        that.particles.alpha[i] = 0.0;
      }else{
          var angle = message.angle_min + i * message.angle_increment;
          that.particles.points[i] = new THREE.Vector3( range * Math.cos(angle), range * Math.sin(angle), 0.0 );
          that.particles.alpha[i] = 1.0;
      }
      that.particles.colors[ i ] = new THREE.Color( that.color );
    }

    finishedUpdate(that.particles, n);
  });
};
ROS3D.LaserScan.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A set of particles. Used by PointCloud2.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * tfClient - the TF client handle to use
 *  * texture - (optional) Image url for a texture to use for the points. Defaults to a single white pixel.
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.Particles = function(options) {
  options = options || {};
  this.tfClient = options.tfClient;
  var texture = options.texture || 'https://upload.wikimedia.org/wikipedia/commons/a/a2/Pixel-white.png';
  var size = options.size || 0.05;
  this.max_pts = options.max_pts || 10000;
  this.first_size = null;
  this.prev_pts = 0;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.vertex_shader = [
    'attribute vec3 customColor;',
    'attribute float alpha;',
    'varying vec3 vColor;',
    'varying float falpha;',
    'void main() ',
    '{',
    '    vColor = customColor; // set color associated to vertex; use later in fragment shader',
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );',
    '    falpha = alpha; ',
    '',
    '    // option (1): draw particles at constant size on screen',
    '    // gl_PointSize = size;',
    '    // option (2): scale particles as objects in 3D space',
    '    gl_PointSize = ', size, '* ( 300.0 / length( mvPosition.xyz ) );',
    '    gl_Position = projectionMatrix * mvPosition;',
    '}'
    ].join('\n');

  this.fragment_shader = [
    'uniform sampler2D texture;',
    'varying vec3 vColor; // colors associated to vertices; assigned by vertex shader',
    'varying float falpha;',
    'void main() ',
    '{',
    '    // calculates a color for the particle',
    '    gl_FragColor = vec4( vColor, falpha );',
    '    // sets particle texture to desired color',
    '    gl_FragColor = gl_FragColor * texture2D( texture, gl_PointCoord );',
    '}'
    ].join('\n');

    this.geom = new THREE.Geometry();
    for(var i=0;i<this.max_pts;i++){
        this.geom.vertices.push(new THREE.Vector3( ));
    }

    var customUniforms =
    {
        texture:   { type: 't', value: THREE.ImageUtils.loadTexture( texture ) },
    };

    this.attribs =
    {
        customColor:   { type: 'c', value: [] },
        alpha:         { type: 'f', value: [] }
    };

    this.shaderMaterial = new THREE.ShaderMaterial(
    {
        uniforms:          customUniforms,
        attributes:        this.attribs,
        vertexShader:      this.vertex_shader,
        fragmentShader:    this.fragment_shader,
        transparent: true, alphaTest: 0.5
    });

    this.ps = new THREE.ParticleSystem( this.geom, this.shaderMaterial );
    this.sn = null;

    this.points = this.geom.vertices;
    this.colors = this.attribs.customColor.value;
    this.alpha =  this.attribs.alpha.value;

};

function setFrame(particles, frame)
{
    if(particles.sn===null){
        particles.sn = new ROS3D.SceneNode({
            frameID : frame,
            tfClient : particles.tfClient,
            object : particles.ps
        });

        particles.rootObject.add(particles.sn);
    }
}

function finishedUpdate(particles, n)
{
    if(particles.first_size === null){
        particles.first_size = n;
        particles.max_pts = Math.max(particles.max_pts, n);
    }

    for(var i=n; i<particles.prev_pts; i++){
        particles.alpha[i] = 0.0;
    }
    particles.prev_pts = n;

    particles.geom.verticesNeedUpdate = true;
    particles.attribs.customColor.needsUpdate = true;
    particles.attribs.alpha.needsUpdate = true;

    if(n>particles.max_pts){
        throw 'Attempted to draw more points than max_pts allows';
    }
}

/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

function read_point(msg, index, buffer){
    var pt = [];
    var base = msg.point_step * index;
    var n = 4;
    var ar = new Uint8Array(n);
    for(var fi=0; fi<msg.fields.length; fi++){
        var si = base + msg.fields[fi].offset;
        for(var i=0; i<n; i++){
            ar[i] = buffer[si + i];
        }

        var dv = new DataView(ar.buffer);

        if( msg.fields[fi].name === 'rgb' ){
            pt[ 'rgb' ] =dv.getInt32(0, 1);
        }else{
            pt[ msg.fields[fi].name ] = dv.getFloat32(0, 1);
        }
    }
    return pt;
}

var BASE64 = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=';
function decode64(x) {
    var a = [], z = 0, bits = 0;

    for (var i = 0, len = x.length; i < len; i++) {
      z += BASE64.indexOf( x[i] );
      bits += 6;
      if(bits>=8){
          bits -= 8;
          a.push(z >> bits);
          z = z & (Math.pow(2, bits)-1);
      }
      z = z << 6;
    }
    return a;
}

/**
 * A PointCloud2 client that listens to a given topic and displays the points.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * texture - (optional) Image url for a texture to use for the points. Defaults to a single white pixel.
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.PointCloud2 = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/points';
  var that = this;

  this.particles = new ROS3D.Particles(options);

  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'sensor_msgs/PointCloud2'
  });

  rosTopic.subscribe(function(message) {
    setFrame(that.particles, message.header.frame_id);

    var n = message.height*message.width;
    var buffer;
    if(message.data.buffer){
      buffer = message.data.buffer;
    }else{
      buffer = decode64(message.data);
    }
    for(var i=0;i<n;i++){
      var pt = read_point(message, i, buffer);
      that.particles.points[i] = new THREE.Vector3( pt['x'], pt['y'], pt['z'] );
      that.particles.colors[ i ] = new THREE.Color( pt['rgb'] );
      that.particles.alpha[i] = 1.0;
    }

    finishedUpdate(that.particles, n);
  });
};
ROS3D.PointCloud2.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF can be used to load a ROSLIB.UrdfModel and its associated models into a 3D object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * urdfModel - the ROSLIB.UrdfModel to load
 *   * tfClient - the TF client handle to use
 *   * path (optional) - the base path to the associated Collada models that will be loaded
 *   * tfPrefix (optional) - the TF prefix to used for multi-robots
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.Urdf = function(options) {
  options = options || {};
  var urdfModel = options.urdfModel;
  var path = options.path || '/';
  var tfClient = options.tfClient;
  var tfPrefix = options.tfPrefix || '';
  var loader = options.loader || ROS3D.COLLADA_LOADER_2;

  THREE.Object3D.call(this);

  // load all models
  var links = urdfModel.links;
  for ( var l in links) {
    var link = links[l];
    for( var i=0; i<link.visuals.length; i++ ) {
      var visual = link.visuals[i];
      if (visual && visual.geometry) {
        // Save frameID
        var frameID = tfPrefix + '/' + link.name;
        // Save color material
        var colorMaterial = null;
        if (visual.material && visual.material.color) {
          var color = visual.material && visual.material.color;
          colorMaterial = ROS3D.makeColorMaterial(color.r, color.g, color.b, color.a);
        }
        if (visual.geometry.type === ROSLIB.URDF_MESH) {
          var uri = visual.geometry.filename;
          // strips package://
          var tmpIndex = uri.indexOf('package://');
          if (tmpIndex !== -1) {
            uri = uri.substr(tmpIndex + ('package://').length);
          }
          var fileType = uri.substr(-4).toLowerCase();

          // ignore mesh files which are not in Collada or STL format
          if (fileType === '.dae' || fileType === '.stl') {
            // create the model
            var mesh = new ROS3D.MeshResource({
              path : path,
              resource : uri,
              loader : loader,
              material : colorMaterial
            });

            // check for a scale
            if(link.visuals[i].geometry.scale) {
              mesh.scale = new THREE.Vector3(
                  visual.geometry.scale.x,
                  visual.geometry.scale.y,
                  visual.geometry.scale.z
                  );
            }

            // create a scene node with the model
            var sceneNode = new ROS3D.SceneNode({
              frameID : frameID,
                pose : visual.origin,
                tfClient : tfClient,
                object : mesh
            });
            this.add(sceneNode);
          } else {
            console.warn('Could not load geometry mesh: '+uri);
          }
        } else {
          if (!colorMaterial) {
            colorMaterial = ROS3D.makeColorMaterial(0, 0, 0, 1);
          }
          var shapeMesh;
          // Create a shape
          switch (visual.geometry.type) {
            case ROSLIB.URDF_BOX:
              var dimension = visual.geometry.dimension;
              var cube = new THREE.CubeGeometry(dimension.x, dimension.y, dimension.z);
              shapeMesh = new THREE.Mesh(cube, colorMaterial);
              break;
            case ROSLIB.URDF_CYLINDER:
              var radius = visual.geometry.radius;
              var length = visual.geometry.length;
              var cylinder = new THREE.CylinderGeometry(radius, radius, length, 16, 1, false);
              shapeMesh = new THREE.Mesh(cylinder, colorMaterial);
              shapeMesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
              break;
            case ROSLIB.URDF_SPHERE:
              var sphere = new THREE.SphereGeometry(visual.geometry.radius, 16);
              shapeMesh = new THREE.Mesh(sphere, colorMaterial);
              break;
          }
          // Create a scene node with the shape
          var scene = new ROS3D.SceneNode({
            frameID: frameID,
              pose: visual.origin,
              tfClient: tfClient,
              object: shapeMesh
          });
          this.add(scene);
        }
      }
    }
  }
};
ROS3D.Urdf.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF client can be used to load a URDF and its associated models into a 3D object from the ROS
 * parameter server.
 *
 * Emits the following events:
 *
 * * 'change' - emited after the URDF and its meshes have been loaded into the root object
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * param (optional) - the paramter to load the URDF from, like 'robot_description'
 *   * tfClient - the TF client handle to use
 *   * path (optional) - the base path to the associated Collada models that will be loaded
 *   * rootObject (optional) - the root object to add this marker to
 *   * tfPrefix (optional) - the TF prefix to used for multi-robots
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.UrdfClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var param = options.param || 'robot_description';
  this.path = options.path || '/';
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var tfPrefix = options.tfPrefix || '';
  var loader = options.loader || ROS3D.COLLADA_LOADER_2;

  // get the URDF value from ROS
  var getParam = new ROSLIB.Param({
    ros : ros,
    name : param
  });
  getParam.get(function(string) {
    // hand off the XML string to the URDF model
    var urdfModel = new ROSLIB.UrdfModel({
      string : string
    });

    // load all models
    that.rootObject.add(new ROS3D.Urdf({
      urdfModel : urdfModel,
      path : that.path,
      tfClient : that.tfClient,
      tfPrefix : tfPrefix,
      loader : loader
    }));
  });
};

/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A SceneNode can be used to keep track of a 3D object with respect to a ROS frame within a scene.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * tfClient - a handle to the TF client
 *  * frameID - the frame ID this object belongs to
 *  * pose (optional) - the pose associated with this object
 *  * object - the THREE 3D object to be rendered
 */
ROS3D.SceneNode = function(options) {
  options = options || {};
  var that = this;
  var tfClient = options.tfClient;
  var frameID = options.frameID;
  var object = options.object;
  this.pose = options.pose || new ROSLIB.Pose();

  THREE.Object3D.call(this);

  // add the model
  this.add(object);

  // set the inital pose
  this.updatePose(this.pose);

  // listen for TF updates
  tfClient.subscribe(frameID, function(msg) {

    // apply the transform
    var tf = new ROSLIB.Transform(msg);
    var poseTransformed = new ROSLIB.Pose(that.pose);
    poseTransformed.applyTransform(tf);

    // update the world
    that.updatePose(poseTransformed);
  });
};
ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the pose of the associated model.
 *
 * @param pose - the pose to update with
 */
ROS3D.SceneNode.prototype.updatePose = function(pose) {
  this.position.set( pose.position.x, pose.position.y, pose.position.z );
  this.quaternion.set(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  this.updateMatrixWorld(true);
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 */

/**
 * A Viewer can be used to render an interactive 3D scene to a HTML5 canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * divID - the ID of the div to place the viewer in
 *  * width - the initial width, in pixels, of the canvas
 *  * height - the initial height, in pixels, of the canvas
 *  * background (optional) - the color to render the background, like '#efefef'
 *  * antialias (optional) - if antialiasing should be used
 *  * intensity (optional) - the lighting intensity setting to use
 *  * cameraPosition (optional) - the starting position of the camera
 */
ROS3D.Viewer = function(options) {
  var that = this;
  options = options || {};
  var divID = options.divID;
  var width = options.width;
  var height = options.height;
  var background = options.background || '#111111';
  var antialias = options.antialias;
  var intensity = options.intensity || 0.66;
  var near = options.near || 0.01;
  var far = options.far || 1000;
  var cameraPosition = options.cameraPose || {
    x : 3,
    y : 3,
    z : 3
  };
  var cameraZoomSpeed = options.cameraZoomSpeed || 0.5;

  // create the canvas to render to
  this.renderer = new THREE.WebGLRenderer({
    antialias : antialias
  });
  this.renderer.setClearColor(parseInt(background.replace('#', '0x'), 16), 1.0);
  this.renderer.sortObjects = false;
  this.renderer.setSize(width, height);
  this.renderer.shadowMapEnabled = false;
  this.renderer.autoClear = false;

  // create the global scene
  this.scene = new THREE.Scene();

  // create the global camera
  this.camera = new THREE.PerspectiveCamera(40, width / height, near, far);
  this.camera.position.x = cameraPosition.x;
  this.camera.position.y = cameraPosition.y;
  this.camera.position.z = cameraPosition.z;
  // add controls to the camera
  this.cameraControls = new ROS3D.OrbitControls({
    scene : this.scene,
    camera : this.camera
  });
  this.cameraControls.userZoomSpeed = cameraZoomSpeed;

  // lights
  this.scene.add(new THREE.AmbientLight(0x555555));
  this.directionalLight = new THREE.DirectionalLight(0xffffff, intensity);
  this.scene.add(this.directionalLight);

  // propagates mouse events to three.js objects
  this.selectableObjects = new THREE.Object3D();
  this.scene.add(this.selectableObjects);
  var mouseHandler = new ROS3D.MouseHandler({
    renderer : this.renderer,
    camera : this.camera,
    rootObject : this.selectableObjects,
    fallbackTarget : this.cameraControls
  });

  // highlights the receiver of mouse events
  this.highlighter = new ROS3D.Highlighter({
    mouseHandler : mouseHandler
  });

  /**
   * Renders the associated scene to the viewer.
   */
  function draw() {
    // update the controls
    that.cameraControls.update();

    // put light to the top-left of the camera
    that.directionalLight.position = that.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    that.directionalLight.position.normalize();

    // set the scene
    that.renderer.clear(true, true, true);
    that.renderer.render(that.scene, that.camera);

    // render any mouseovers
    that.highlighter.renderHighlight(that.renderer, that.scene, that.camera);

    // draw the frame
    requestAnimationFrame(draw);
  }

  // add the renderer to the page
  document.getElementById(divID).appendChild(this.renderer.domElement);

  // begin the animation
  draw();
};

/**
 * Add the given THREE Object3D to the global scene in the viewer.
 *
 * @param object - the THREE Object3D to add
 * @param selectable (optional) - if the object should be added to the selectable list
 */
ROS3D.Viewer.prototype.addObject = function(object, selectable) {
  if (selectable) {
    this.selectableObjects.add(object);
  } else {
    this.scene.add(object);
  }
};

/**
 * Resize 3D viewer
 *
 * @param width - new width value
 * @param height - new height value
 */
ROS3D.Viewer.prototype.resize = function(width, height) {
  this.camera.aspect = width / height;
  this.camera.updateProjectionMatrix();
  this.renderer.setSize(width, height);
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A mouseover highlighter for 3D objects in the scene.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * mouseHandler - the handler for the mouseover and mouseout events
 */
ROS3D.Highlighter = function(options) {
  options = options || {};
  var mouseHandler = options.mouseHandler;
  this.hoverObjs = [];

  // bind the mouse events
  mouseHandler.addEventListener('mouseover', this.onMouseOver.bind(this));
  mouseHandler.addEventListener('mouseout', this.onMouseOut.bind(this));
};

/**
 * Add the current target of the mouseover to the hover list.
 *
 * @param event - the event that contains the target of the mouseover
 */
ROS3D.Highlighter.prototype.onMouseOver = function(event) {
  this.hoverObjs.push(event.currentTarget);
};

/**
 * Remove the current target of the mouseover from the hover list.
 *
 * @param event - the event that contains the target of the mouseout
 */
ROS3D.Highlighter.prototype.onMouseOut = function(event) {
  this.hoverObjs.splice(this.hoverObjs.indexOf(event.currentTarget), 1);
};

/**
 * Add all corresponding webgl objects in the given scene and add them to the given render list.
 *
 * @param scene - the scene to check for webgl objects
 * @param objects - the objects list to check
 * @param renderList - the list to add to
 */
ROS3D.Highlighter.prototype.getWebglObjects = function(scene, objects, renderList) {
  var objlist = scene.__webglObjects;
  // get corresponding webgl objects
  for ( var c = 0; c < objects.length; c++) {
    if (objects[c]) {
      for ( var o = objlist.length - 1; o >= 0; o--) {
        if (objlist[o].object === objects[c]) {
          renderList.push(objlist[o]);
          break;
        }
      }
      // recurse into children
      this.getWebglObjects(scene, objects[c].children, renderList);
    }
  }
};

/**
 * Render highlighted objects in the scene.
 *
 * @param renderer - the renderer to use
 * @param scene - the scene to use
 * @param camera - the camera to use
 */
ROS3D.Highlighter.prototype.renderHighlight = function(renderer, scene, camera) {
  // get webgl objects
  var renderList = [];
  this.getWebglObjects(scene, this.hoverObjs, renderList);

  // define highlight material
  scene.overrideMaterial = new THREE.MeshBasicMaterial({
    fog : false,
    opacity : 0.5,
    depthTest : true,
    depthWrite : false,
    polygonOffset : true,
    polygonOffsetUnits : -1,
    side : THREE.DoubleSide
  });

  // swap render lists, render, undo
  var oldWebglObjects = scene.__webglObjects;
  scene.__webglObjects = renderList;

  renderer.render(scene, camera);

  scene.__webglObjects = oldWebglObjects;
  scene.overrideMaterial = null;
};

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A handler for mouse events within a 3D viewer.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * renderer - the main renderer
 *   * camera - the main camera in the scene
 *   * rootObject - the root object to check for mouse events
 *   * fallbackTarget - the fallback target, e.g., the camera controls
 */
ROS3D.MouseHandler = function(options) {
  THREE.EventDispatcher.call(this);
  this.renderer = options.renderer;
  this.camera = options.camera;
  this.rootObject = options.rootObject;
  this.fallbackTarget = options.fallbackTarget;
  this.lastTarget = this.fallbackTarget;
  this.dragging = false;
  this.projector = new THREE.Projector();

  // listen to DOM events
  var eventNames = [ 'contextmenu', 'click', 'dblclick', 'mouseout', 'mousedown', 'mouseup',
      'mousemove', 'mousewheel', 'DOMMouseScroll', 'touchstart', 'touchend', 'touchcancel',
      'touchleave', 'touchmove' ];
  this.listeners = {};

  // add event listeners for the associated mouse events
  eventNames.forEach(function(eventName) {
    this.listeners[eventName] = this.processDomEvent.bind(this);
    this.renderer.domElement.addEventListener(eventName, this.listeners[eventName], false);
  }, this);
};

/**
 * Process the particular DOM even that has occurred based on the mouse's position in the scene.
 *
 * @param domEvent - the DOM event to process
 */
ROS3D.MouseHandler.prototype.processDomEvent = function(domEvent) {
  // don't deal with the default handler
  domEvent.preventDefault();

  // compute normalized device coords and 3D mouse ray
  var target = domEvent.target;
  var rect = target.getBoundingClientRect();
  var pos_x, pos_y;

  if(domEvent.type.indexOf('touch') !== -1) {
	pos_x = domEvent.changedTouches[0].clientX;
	pos_y = domEvent.changedTouches[0].clientY;
  }
  else {
	pos_x = domEvent.clientX;
	pos_y = domEvent.clientY;
  }
  var left = pos_x - rect.left - target.clientLeft + target.scrollLeft;
  var top = pos_y - rect.top - target.clientTop + target.scrollTop;
  var deviceX = left / target.clientWidth * 2 - 1;
  var deviceY = -top / target.clientHeight * 2 + 1;
  var vector = new THREE.Vector3(deviceX, deviceY, 0.5);
  this.projector.unprojectVector(vector, this.camera);
  // use the THREE raycaster
  var mouseRaycaster = new THREE.Raycaster(this.camera.position.clone(), vector.sub(
      this.camera.position).normalize());
  mouseRaycaster.linePrecision = 0.001;
  var mouseRay = mouseRaycaster.ray;

  // make our 3d mouse event
  var event3D = {
    mousePos : new THREE.Vector2(deviceX, deviceY),
    mouseRay : mouseRay,
    domEvent : domEvent,
    camera : this.camera,
    intersection : this.lastIntersection
  };

  // if the mouse leaves the dom element, stop everything
  if (domEvent.type === 'mouseout') {
    if (this.dragging) {
      this.notify(this.lastTarget, 'mouseup', event3D);
      this.dragging = false;
    }
    this.notify(this.lastTarget, 'mouseout', event3D);
    this.lastTarget = null;
    return;
  }

  // if the touch leaves the dom element, stop everything
  if (domEvent.type === 'touchleave' || domEvent.type === 'touchend') {
    if (this.dragging) {
      this.notify(this.lastTarget, 'mouseup', event3D);
      this.dragging = false;
    }
    this.notify(this.lastTarget, 'touchend', event3D);
    this.lastTarget = null;
    return;
  }

  // while the user is holding the mouse down, stay on the same target
  if (this.dragging) {
    this.notify(this.lastTarget, domEvent.type, event3D);
    // for check for right or left mouse button
    if ((domEvent.type === 'mouseup' && domEvent.button === 2) || domEvent.type === 'click' || domEvent.type === 'touchend') {
      this.dragging = false;
    }
    return;
  }

  // in the normal case, we need to check what is under the mouse
  target = this.lastTarget;
  var intersections = [];
  intersections = mouseRaycaster.intersectObject(this.rootObject, true);
  if (intersections.length > 0) {
    target = intersections[0].object;
    event3D.intersection = this.lastIntersection = intersections[0];
  } else {
    target = this.fallbackTarget;
  }

  // if the mouse moves from one object to another (or from/to the 'null' object), notify both
  if (target !== this.lastTarget && domEvent.type.match(/mouse/)) {
    var eventAccepted = this.notify(target, 'mouseover', event3D);
    if (eventAccepted) {
      this.notify(this.lastTarget, 'mouseout', event3D);
    } else {
      // if target was null or no target has caught our event, fall back
      target = this.fallbackTarget;
      if (target !== this.lastTarget) {
        this.notify(target, 'mouseover', event3D);
        this.notify(this.lastTarget, 'mouseout', event3D);
      }
    }
  }

  // if the finger moves from one object to another (or from/to the 'null' object), notify both
  if (target !== this.lastTarget && domEvent.type.match(/touch/)) {
    var toucheventAccepted = this.notify(target, domEvent.type, event3D);
    if (toucheventAccepted) {
      this.notify(this.lastTarget, 'touchleave', event3D);
      this.notify(this.lastTarget, 'touchend', event3D);
    } else {
      // if target was null or no target has caught our event, fall back
      target = this.fallbackTarget;
      if (target !== this.lastTarget) {
        this.notify(this.lastTarget, 'touchmove', event3D);
        this.notify(this.lastTarget, 'touchend', event3D);
      }
    }
  }

  // pass through event
  this.notify(target, domEvent.type, event3D);
  if (domEvent.type === 'mousedown' || domEvent.type === 'touchstart' || domEvent.type === 'touchmove') {
    this.dragging = true;
  }
  this.lastTarget = target;
};

/**
 * Notify the listener of the type of event that occurred.
 *
 * @param target - the target of the event
 * @param type - the type of event that occurred
 * @param event3D - the 3D mouse even information
 * @returns if an event was canceled
 */
ROS3D.MouseHandler.prototype.notify = function(target, type, event3D) {
  // ensure the type is set
  event3D.type = type;

  // make the event cancelable
  event3D.cancelBubble = false;
  event3D.stopPropagation = function() {
    event3D.cancelBubble = true;
  };
  // walk up graph until event is canceled or root node has been reached
  event3D.currentTarget = target;
  while (event3D.currentTarget) {
    // try to fire event on object
    if (event3D.currentTarget.dispatchEvent
        && event3D.currentTarget.dispatchEvent instanceof Function) {
      event3D.currentTarget.dispatchEvent(event3D);
      if (event3D.cancelBubble) {
        this.dispatchEvent(event3D);
        return true;
      }
    }
    // walk up
    event3D.currentTarget = event3D.currentTarget.parent;
  }
  return false;
};

THREE.EventDispatcher.prototype.apply( ROS3D.MouseHandler.prototype );

/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Xueqiao Xu - xueqiaoxu@gmail.com
 * @author Mr.doob - http://mrdoob.com
 * @author AlteredQualia - http://alteredqualia.com
 */

/**
 * Behaves like THREE.OrbitControls, but uses right-handed coordinates and z as up vector.
 *
 * @constructor
 * @param scene - the global scene to use
 * @param camera - the camera to use
 * @param userZoomSpeed (optional) - the speed for zooming
 * @param userRotateSpeed (optional) - the speed for rotating
 * @param autoRotate (optional) - if the orbit should auto rotate
 * @param autoRotate (optional) - the speed for auto rotating
 */
ROS3D.OrbitControls = function(options) {
  THREE.EventDispatcher.call(this);
  var that = this;
  options = options || {};
  var scene = options.scene;
  this.camera = options.camera;
  this.center = new THREE.Vector3();
  this.userZoom = true;
  this.userZoomSpeed = options.userZoomSpeed || 1.0;
  this.userRotate = true;
  this.userRotateSpeed = options.userRotateSpeed || 1.0;
  this.autoRotate = options.autoRotate;
  this.autoRotateSpeed = options.autoRotateSpeed || 2.0;

  // In ROS, z is pointing upwards
  this.camera.up = new THREE.Vector3(0, 0, 1);

  // internals
  var pixelsPerRound = 1800;
  var touchMoveThreshold = 10;
  var rotateStart = new THREE.Vector2();
  var rotateEnd = new THREE.Vector2();
  var rotateDelta = new THREE.Vector2();
  var zoomStart = new THREE.Vector2();
  var zoomEnd = new THREE.Vector2();
  var zoomDelta = new THREE.Vector2();
  var moveStartCenter = new THREE.Vector3();
  var moveStartNormal = new THREE.Vector3();
  var moveStartPosition = new THREE.Vector3();
  var moveStartIntersection = new THREE.Vector3();
  var touchStartPosition = new Array(2);
  var touchMoveVector = new Array(2);
  this.phiDelta = 0;
  this.thetaDelta = 0;
  this.scale = 1;
  this.lastPosition = new THREE.Vector3();
  // internal states
  var STATE = {
    NONE : -1,
    ROTATE : 0,
    ZOOM : 1,
    MOVE : 2
  };
  var state = STATE.NONE;

  // add the axes for the main coordinate frame
  this.axes = new ROS3D.Axes({
    shaftRadius : 0.025,
    headRadius : 0.07,
    headLength : 0.2
  });
  // initially not visible
  scene.add(this.axes);
  this.axes.traverse(function(obj) {
    obj.visible = false;
  });

  /**
   * Handle the mousedown 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onMouseDown(event3D) {
    var event = event3D.domEvent;
    event.preventDefault();

    switch (event.button) {
      case 0:
        state = STATE.ROTATE;
        rotateStart.set(event.clientX, event.clientY);
        break;
      case 1:
        state = STATE.MOVE;

        moveStartNormal = new THREE.Vector3(0, 0, 1);
        var rMat = new THREE.Matrix4().extractRotation(this.camera.matrix);
        moveStartNormal.applyMatrix4(rMat);

        moveStartCenter = that.center.clone();
        moveStartPosition = that.camera.position.clone();
        moveStartIntersection = intersectViewPlane(event3D.mouseRay,
                                                   moveStartCenter,
                                                   moveStartNormal);
        break;
      case 2:
        state = STATE.ZOOM;
        zoomStart.set(event.clientX, event.clientY);
        break;
    }

    this.showAxes();
  }

  /**
   * Handle the mousemove 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onMouseMove(event3D) {
    var event = event3D.domEvent;
    if (state === STATE.ROTATE) {

      rotateEnd.set(event.clientX, event.clientY);
      rotateDelta.subVectors(rotateEnd, rotateStart);

      that.rotateLeft(2 * Math.PI * rotateDelta.x / pixelsPerRound * that.userRotateSpeed);
      that.rotateUp(2 * Math.PI * rotateDelta.y / pixelsPerRound * that.userRotateSpeed);

      rotateStart.copy(rotateEnd);
      this.showAxes();
    } else if (state === STATE.ZOOM) {
      zoomEnd.set(event.clientX, event.clientY);
      zoomDelta.subVectors(zoomEnd, zoomStart);

      if (zoomDelta.y > 0) {
        that.zoomIn();
      } else {
        that.zoomOut();
      }

      zoomStart.copy(zoomEnd);
      this.showAxes();

    } else if (state === STATE.MOVE) {
      var intersection = intersectViewPlane(event3D.mouseRay, that.center, moveStartNormal);

      if (!intersection) {
        return;
      }

      var delta = new THREE.Vector3().subVectors(moveStartIntersection.clone(), intersection
          .clone());

      that.center.addVectors(moveStartCenter.clone(), delta.clone());
      that.camera.position.addVectors(moveStartPosition.clone(), delta.clone());
      that.update();
      that.camera.updateMatrixWorld();
      this.showAxes();
    }
  }

  /**
   * Used to track the movement during camera movement.
   *
   * @param mouseRay - the mouse ray to intersect with
   * @param planeOrigin - the origin of the plane
   * @param planeNormal - the normal of the plane
   * @returns the intersection
   */
  function intersectViewPlane(mouseRay, planeOrigin, planeNormal) {

    var vector = new THREE.Vector3();
    var intersection = new THREE.Vector3();

    vector.subVectors(planeOrigin, mouseRay.origin);
    var dot = mouseRay.direction.dot(planeNormal);

    // bail if ray and plane are parallel
    if (Math.abs(dot) < mouseRay.precision) {
      return null;
    }

    // calc distance to plane
    var scalar = planeNormal.dot(vector) / dot;

    intersection = mouseRay.direction.clone().multiplyScalar(scalar);
    return intersection;
  }

  /**
   * Handle the mouseup 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onMouseUp(event3D) {
    if (!that.userRotate) {
      return;
    }

    state = STATE.NONE;
  }

  /**
   * Handle the mousewheel 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onMouseWheel(event3D) {
    if (!that.userZoom) {
      return;
    }

    var event = event3D.domEvent;
    // wheelDelta --> Chrome, detail --> Firefox
    var delta;
    if (typeof (event.wheelDelta) !== 'undefined') {
      delta = event.wheelDelta;
    } else {
      delta = -event.detail;
    }
    if (delta > 0) {
      that.zoomIn();
    } else {
      that.zoomOut();
    }

    this.showAxes();
  }

  /**
   * Handle the touchdown 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onTouchDown(event3D) {
    var event = event3D.domEvent;
    switch (event.touches.length) {
      case 1:
        state = STATE.ROTATE;
        rotateStart.set(event.touches[0].pageX - window.scrollX,
                        event.touches[0].pageY - window.scrollY);
        break;
      case 2:
        state = STATE.NONE;
        /* ready for move */
        moveStartNormal = new THREE.Vector3(0, 0, 1);
        var rMat = new THREE.Matrix4().extractRotation(this.camera.matrix);
        moveStartNormal.applyMatrix4(rMat);
        moveStartCenter = that.center.clone();
        moveStartPosition = that.camera.position.clone();
        moveStartIntersection = intersectViewPlane(event3D.mouseRay,
                                                   moveStartCenter,
                                                   moveStartNormal);
        touchStartPosition[0] = new THREE.Vector2(event.touches[0].pageX,
                                                  event.touches[0].pageY);
        touchStartPosition[1] = new THREE.Vector2(event.touches[1].pageX,
                                                  event.touches[1].pageY);
        touchMoveVector[0] = new THREE.Vector2(0, 0);
        touchMoveVector[1] = new THREE.Vector2(0, 0);
        break;
    }

    this.showAxes();

    event.preventDefault();
  }

  /**
   * Handle the touchmove 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onTouchMove(event3D) {
    var event = event3D.domEvent;
    if (state === STATE.ROTATE) {

      rotateEnd.set(event.touches[0].pageX - window.scrollX, event.touches[0].pageY - window.scrollY);
      rotateDelta.subVectors(rotateEnd, rotateStart);

      that.rotateLeft(2 * Math.PI * rotateDelta.x / pixelsPerRound * that.userRotateSpeed);
      that.rotateUp(2 * Math.PI * rotateDelta.y / pixelsPerRound * that.userRotateSpeed);

      rotateStart.copy(rotateEnd);
      this.showAxes();
    } else {
      touchMoveVector[0].set(touchStartPosition[0].x - event.touches[0].pageX,
                             touchStartPosition[0].y - event.touches[0].pageY);
      touchMoveVector[1].set(touchStartPosition[1].x - event.touches[1].pageX,
                             touchStartPosition[1].y - event.touches[1].pageY);
      if (touchMoveVector[0].lengthSq() > touchMoveThreshold &&
          touchMoveVector[1].lengthSq() > touchMoveThreshold) {
        touchStartPosition[0].set(event.touches[0].pageX,
                                  event.touches[0].pageY);
        touchStartPosition[1].set(event.touches[1].pageX,
                                  event.touches[1].pageY);
        if (touchMoveVector[0].dot(touchMoveVector[1]) > 0 &&
            state !== STATE.ZOOM) {
          state = STATE.MOVE;
        } else if (touchMoveVector[0].dot(touchMoveVector[1]) < 0 &&
                   state !== STATE.MOVE) {
          state = STATE.ZOOM;
        }
        if (state === STATE.ZOOM) {
          var tmpVector = new THREE.Vector2();
          tmpVector.subVectors(touchStartPosition[0],
                               touchStartPosition[1]);
          if (touchMoveVector[0].dot(tmpVector) < 0 &&
              touchMoveVector[1].dot(tmpVector) > 0) {
            that.zoomOut();
          } else if (touchMoveVector[0].dot(tmpVector) > 0 &&
                     touchMoveVector[1].dot(tmpVector) < 0) {
            that.zoomIn();
          }
        }
      }
      if (state === STATE.MOVE) {
        var intersection = intersectViewPlane(event3D.mouseRay,
                                              that.center,
                                              moveStartNormal);
        if (!intersection) {
          return;
        }
        var delta = new THREE.Vector3().subVectors(moveStartIntersection.clone(),
                                                   intersection.clone());
        that.center.addVectors(moveStartCenter.clone(), delta.clone());
        that.camera.position.addVectors(moveStartPosition.clone(), delta.clone());
        that.update();
        that.camera.updateMatrixWorld();
      }

      this.showAxes();

      event.preventDefault();
    }
  }

  function onTouchEnd(event3D) {
    var event = event3D.domEvent;
    if (event.touches.length === 1 &&
        state !== STATE.ROTATE) {
      state = STATE.ROTATE;
      rotateStart.set(event.touches[0].pageX - window.scrollX,
                      event.touches[0].pageY - window.scrollY);
    }
  }

  // add event listeners
  this.addEventListener('mousedown', onMouseDown);
  this.addEventListener('mouseup', onMouseUp);
  this.addEventListener('mousemove', onMouseMove);
  this.addEventListener('touchstart', onTouchDown);
  this.addEventListener('touchmove', onTouchMove);
  this.addEventListener('touchend', onTouchEnd);
  // Chrome/Firefox have different events here
  this.addEventListener('mousewheel', onMouseWheel);
  this.addEventListener('DOMMouseScroll', onMouseWheel);
};

/**
 * Display the main axes for 1 second.
 */
ROS3D.OrbitControls.prototype.showAxes = function() {
  var that = this;

  this.axes.traverse(function(obj) {
    obj.visible = true;
  });
  if (this.hideTimeout) {
    clearTimeout(this.hideTimeout);
  }
  this.hideTimeout = setTimeout(function() {
    that.axes.traverse(function(obj) {
      obj.visible = false;
    });
    that.hideTimeout = false;
  }, 1000);
};

/**
 * Rotate the camera to the left by the given angle.
 *
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateLeft = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.thetaDelta -= angle;
};

/**
 * Rotate the camera to the right by the given angle.
 *
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateRight = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.thetaDelta += angle;
};

/**
 * Rotate the camera up by the given angle.
 *
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateUp = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.phiDelta -= angle;
};

/**
 * Rotate the camera down by the given angle.
 *
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateDown = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.phiDelta += angle;
};

/**
 * Zoom in by the given scale.
 *
 * @param zoomScale (optional) - the scale to zoom in by
 */
ROS3D.OrbitControls.prototype.zoomIn = function(zoomScale) {
  if (zoomScale === undefined) {
    zoomScale = Math.pow(0.95, this.userZoomSpeed);
  }
  this.scale /= zoomScale;
};

/**
 * Zoom out by the given scale.
 *
 * @param zoomScale (optional) - the scale to zoom in by
 */
ROS3D.OrbitControls.prototype.zoomOut = function(zoomScale) {
  if (zoomScale === undefined) {
    zoomScale = Math.pow(0.95, this.userZoomSpeed);
  }
  this.scale *= zoomScale;
};

/**
 * Update the camera to the current settings.
 */
ROS3D.OrbitControls.prototype.update = function() {
  // x->y, y->z, z->x
  var position = this.camera.position;
  var offset = position.clone().sub(this.center);

  // angle from z-axis around y-axis
  var theta = Math.atan2(offset.y, offset.x);

  // angle from y-axis
  var phi = Math.atan2(Math.sqrt(offset.y * offset.y + offset.x * offset.x), offset.z);

  if (this.autoRotate) {
    this.rotateLeft(2 * Math.PI / 60 / 60 * this.autoRotateSpeed);
  }

  theta += this.thetaDelta;
  phi += this.phiDelta;

  // restrict phi to be between EPS and PI-EPS
  var eps = 0.000001;
  phi = Math.max(eps, Math.min(Math.PI - eps, phi));

  var radius = offset.length();
  offset.y = radius * Math.sin(phi) * Math.sin(theta);
  offset.z = radius * Math.cos(phi);
  offset.x = radius * Math.sin(phi) * Math.cos(theta);
  offset.multiplyScalar(this.scale);

  position.copy(this.center).add(offset);

  this.camera.lookAt(this.center);

  radius = offset.length();
  this.axes.position = this.center.clone();
  this.axes.scale.x = this.axes.scale.y = this.axes.scale.z = radius * 0.05;
  this.axes.updateMatrixWorld(true);

  this.thetaDelta = 0;
  this.phiDelta = 0;
  this.scale = 1;

  if (this.lastPosition.distanceTo(this.camera.position) > 0) {
    this.dispatchEvent({
      type : 'change'
    });
    this.lastPosition.copy(this.camera.position);
  }
};

THREE.EventDispatcher.prototype.apply( ROS3D.OrbitControls.prototype );
