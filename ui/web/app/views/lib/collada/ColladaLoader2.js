(function() {
  var ColladaAnimation, ColladaAnimationTarget, ColladaAsset, ColladaCamera, ColladaCameraParam, ColladaChannel, ColladaColorOrTexture, ColladaController, ColladaEffect, ColladaEffectSampler, ColladaEffectSurface, ColladaEffectTechnique, ColladaFile, ColladaFxLink, ColladaGeometry, ColladaImage, ColladaInput, ColladaInstanceCamera, ColladaInstanceController, ColladaInstanceGeometry, ColladaInstanceLight, ColladaInstanceMaterial, ColladaJoints, ColladaLight, ColladaLightParam, ColladaLoader2, ColladaMaterial, ColladaNodeTransform, ColladaSampler, ColladaSidLink, ColladaSkin, ColladaSource, ColladaTriangles, ColladaUrlLink, ColladaVertexWeights, ColladaVertices, ColladaVisualScene, ColladaVisualSceneNode, TO_RADIANS, ThreejsAnimationChannel, ThreejsMaterialMap, ThreejsSkeletonBone, getNodeInfo, graphNodeString, indentString, _checkMatrix4, _colorToHex, _fillMatrix4ColumnMajor, _fillMatrix4RowMajor, _floatsToMatrix4ColumnMajor, _floatsToMatrix4RowMajor, _floatsToVec3, _strToBools, _strToColor, _strToFloats, _strToInts, _strToStrings,
    __hasProp = Object.prototype.hasOwnProperty,
    __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor; child.__super__ = parent.prototype; return child; };

  indentString = function(count, str) {
    var i, output;
    output = "";
    for (i = 1; i <= count; i += 1) {
      output += "    ";
    }
    output += str;
    return output;
  };

  graphNodeString = function(indent, str) {
    return indentString(indent, "|-" + str);
  };

  getNodeInfo = function(node, indent, prefix) {
    if (!(node != null)) return "";
    if (typeof node === "string") {
      return graphNodeString(indent, prefix + ("'" + node + "'\n"));
    }
    if (typeof node === "number") {
      return graphNodeString(indent, prefix + ("" + node + "\n"));
    }
    if (typeof node === "boolean") {
      return graphNodeString(indent, prefix + ("" + node + "\n"));
    }
    if (node.getInfo != null) return node.getInfo(indent, prefix);
    return graphNodeString(indent, prefix + "<unknown data type>\n");
  };

  ColladaUrlLink = (function() {

    function ColladaUrlLink(url) {
      this.url = url.trim().replace(/^#/, "");
      this.object = null;
    }

    ColladaUrlLink.prototype.getInfo = function(indent, prefix) {
      return graphNodeString(indent, prefix + ("<urlLink url='" + this.url + "'>\n"));
    };

    return ColladaUrlLink;

  })();

  ColladaFxLink = (function() {

    function ColladaFxLink(url, scope) {
      this.url = url;
      this.scope = scope;
      this.object = null;
    }

    ColladaFxLink.prototype.getInfo = function(indent, prefix) {
      return graphNodeString(indent, prefix + ("<fxLink url='" + this.url + "'>\n"));
    };

    return ColladaFxLink;

  })();

  ColladaSidLink = (function() {

    function ColladaSidLink(parentId, url) {
      var arrIndices, arrSyntax, dotSyntax, index, lastSid, parts, _i, _len;
      this.url = url;
      this.object = null;
      this.id = null;
      this.sids = [];
      this.member = null;
      this.indices = null;
      this.dotSyntax = false;
      this.arrSyntax = false;
      parts = url.split("/");
      this.id = parts.shift();
      if (this.id === ".") this.id = parentId;
      while (parts.length > 1) {
        this.sids.push(parts.shift());
      }
      if (parts.length > 0) {
        lastSid = parts[0];
        dotSyntax = lastSid.indexOf(".") >= 0;
        arrSyntax = lastSid.indexOf("(") >= 0;
        if (dotSyntax) {
          parts = lastSid.split(".");
          this.sids.push(parts.shift());
          this.member = parts.shift();
          this.dotSyntax = true;
        } else if (arrSyntax) {
          arrIndices = lastSid.split("(");
          this.sids.push(arrIndices.shift());
          this.indices = [];
          for (_i = 0, _len = arrIndices.length; _i < _len; _i++) {
            index = arrIndices[_i];
            this.indices.push(parseInt(index.replace(/\)/, ""), 10));
          }
          this.arrSyntax = true;
        } else {
          this.sids.push(lastSid);
        }
      }
    }

    ColladaSidLink.prototype.getInfo = function(indent, prefix) {
      var output, str;
      str = "<sidLink id='" + this.id + "'";
      if (this.sids.length > 0) {
        str += ", sids='[";
        str += this.sids.join(",");
        str += "]'";
      }
      str += ">\n";
      return output = graphNodeString(indent, prefix + str);
    };

    return ColladaSidLink;

  })();

  ColladaAnimationTarget = (function() {

    function ColladaAnimationTarget() {
      this.animTarget = {};
      this.animTarget.channels = [];
      this.animTarget.activeChannels = [];
      this.animTarget.dataRows = null;
      this.animTarget.dataColumns = null;
    }

    ColladaAnimationTarget.prototype.selectAnimation = function(filter) {
      var channel, i, _len, _ref;
      this.animTarget.activeChannels = [];
      _ref = this.animTarget.channels;
      for (i = 0, _len = _ref.length; i < _len; i++) {
        channel = _ref[i];
        if (filter(channel, i)) this.animTarget.activeChannels.push(channel);
      }
    };

    ColladaAnimationTarget.prototype.selectAnimationById = function(id) {
      this.selectAnimation(function(channel, i) {
        return channel.animation.id === id;
      });
    };

    ColladaAnimationTarget.prototype.selectAnimationByName = function(name) {
      this.selectAnimation(function(channel, i) {
        return channel.animation.name === name;
      });
    };

    ColladaAnimationTarget.prototype.selectAllAnimations = function(index) {
      this.selectAnimation(function(channel, i) {
        return true;
      });
    };

    ColladaAnimationTarget.prototype.applyAnimationKeyframe = function(keyframe) {
      throw new Error("applyAnimationKeyframe() not implemented");
    };

    ColladaAnimationTarget.prototype.initAnimationTarget = function() {
      throw new Error("initAnimationTarget() not implemented");
    };

    ColladaAnimationTarget.prototype.resetAnimation = function() {
      throw new Error("resetAnimation() not implemented");
    };

    return ColladaAnimationTarget;

  })();

  ColladaAsset = (function() {

    function ColladaAsset() {
      this.unit = 1;
      this.upAxis = null;
    }

    ColladaAsset.prototype.getInfo = function(indent, prefix) {
      return graphNodeString(indent, prefix + "<asset>\n");
    };

    return ColladaAsset;

  })();

  ColladaVisualScene = (function() {

    function ColladaVisualScene() {
      this.id = null;
      this.children = [];
      this.sidChildren = [];
    }

    ColladaVisualScene.prototype.getInfo = function(indent, prefix) {
      var child, output, _i, _len, _ref;
      output = graphNodeString(indent, prefix + ("<visualScene id='" + this.id + "'>\n"));
      if (this.children != null) {
        _ref = this.children;
        for (_i = 0, _len = _ref.length; _i < _len; _i++) {
          child = _ref[_i];
          output += getNodeInfo(child, indent + 1, "child ");
        }
      }
      return output;
    };

    return ColladaVisualScene;

  })();

  ColladaVisualSceneNode = (function() {

    function ColladaVisualSceneNode() {
      this.id = null;
      this.sid = null;
      this.name = null;
      this.type = null;
      this.layer = null;
      this.parent = null;
      this.children = [];
      this.sidChildren = [];
      this.transformations = [];
      this.geometries = [];
      this.controllers = [];
      this.lights = [];
      this.cameras = [];
    }

    ColladaVisualSceneNode.prototype.getInfo = function(indent, prefix) {
      var child, output, _i, _j, _k, _l, _len, _len2, _len3, _len4, _len5, _m, _ref, _ref2, _ref3, _ref4, _ref5;
      output = graphNodeString(indent, prefix + ("<visualSceneNode id='" + this.id + "', sid='" + this.sid + "', name='" + this.name + "'>\n"));
      if (this.geometries != null) {
        _ref = this.geometries;
        for (_i = 0, _len = _ref.length; _i < _len; _i++) {
          child = _ref[_i];
          output += getNodeInfo(child, indent + 1, "geometry ");
        }
      }
      if (this.controllers != null) {
        _ref2 = this.controllers;
        for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
          child = _ref2[_j];
          output += getNodeInfo(child, indent + 1, "controller ");
        }
      }
      if (this.lights != null) {
        _ref3 = this.lights;
        for (_k = 0, _len3 = _ref3.length; _k < _len3; _k++) {
          child = _ref3[_k];
          output += getNodeInfo(child, indent + 1, "light ");
        }
      }
      if (this.cameras != null) {
        _ref4 = this.cameras;
        for (_l = 0, _len4 = _ref4.length; _l < _len4; _l++) {
          child = _ref4[_l];
          output += getNodeInfo(child, indent + 1, "camera ");
        }
      }
      if (this.children != null) {
        _ref5 = this.children;
        for (_m = 0, _len5 = _ref5.length; _m < _len5; _m++) {
          child = _ref5[_m];
          output += getNodeInfo(child, indent + 1, "child ");
        }
      }
      return output;
    };

    ColladaVisualSceneNode.prototype.getTransformMatrix = function(result) {
      var temp, transform, _i, _len, _ref;
      temp = new THREE.Matrix4;
      result.identity();
      _ref = this.transformations;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        transform = _ref[_i];
        transform.getTransformMatrix(temp);
        result.multiplyMatrices(result, temp);
      }
    };

    return ColladaVisualSceneNode;

  })();

  ColladaNodeTransform = (function(_super) {

    __extends(ColladaNodeTransform, _super);

    function ColladaNodeTransform() {
      ColladaNodeTransform.__super__.constructor.call(this);
      this.sid = null;
      this.type = null;
      this.data = null;
      this.originalData = null;
      this.node = null;
    }

    ColladaNodeTransform.prototype.getTransformMatrix = function(result) {
      var axis;
      switch (this.type) {
        case "matrix":
          _fillMatrix4RowMajor(this.data, 0, result);
          break;
        case "rotate":
          axis = new THREE.Vector3(this.data[0], this.data[1], this.data[2]);
          result.makeRotationAxis(axis, this.data[3] * TO_RADIANS);
          break;
        case "translate":
          result.makeTranslation(this.data[0], this.data[1], this.data[2]);
          break;
        case "scale":
          result.makeScale(this.data[0], this.data[1], this.data[2]);
          break;
        default:
          throw new Error("transform type '" + this.type + "' not implemented");
      }
    };

    ColladaNodeTransform.prototype.applyAnimationKeyframe = function(keyframe) {
      var channel, i, outputData, _i, _len, _ref, _ref2;
      _ref = this.animTarget.activeChannels;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        channel = _ref[_i];
        outputData = channel.outputData;
        for (i = 0, _ref2 = channel.count - 1; i <= _ref2; i += 1) {
          this.data[channel.offset + i] = outputData[keyframe * channel.stride + i];
        }
      }
    };

    ColladaNodeTransform.prototype.initAnimationTarget = function() {
      var i, x, _len, _ref;
      this.originalData = new Float32Array(this.data.length);
      _ref = this.data;
      for (i = 0, _len = _ref.length; i < _len; i++) {
        x = _ref[i];
        this.originalData[i] = this.data[i];
      }
      switch (this.type) {
        case "matrix":
          this.animTarget.dataColumns = 4;
          this.animTarget.dataRows = 4;
          break;
        case "rotate":
          this.animTarget.dataColumns = 4;
          this.animTarget.dataRows = 1;
          break;
        case "translate":
        case "scale":
          this.animTarget.dataColumns = 3;
          this.animTarget.dataRows = 1;
          break;
        default:
          throw new Error("transform type '" + this.type + "' not implemented");
      }
    };

    ColladaNodeTransform.prototype.resetAnimation = function() {
      var i, x, _len, _ref;
      _ref = this.originalData;
      for (i = 0, _len = _ref.length; i < _len; i++) {
        x = _ref[i];
        this.data[i] = this.originalData[i];
      }
    };

    return ColladaNodeTransform;

  })(ColladaAnimationTarget);

  ColladaInstanceGeometry = (function() {

    function ColladaInstanceGeometry() {
      this.geometry = null;
      this.materials = [];
      this.sidChildren = [];
    }

    ColladaInstanceGeometry.prototype.getInfo = function(indent, prefix) {
      var material, output, _i, _len, _ref;
      output = graphNodeString(indent, prefix + "<instanceGeometry>\n");
      output += getNodeInfo(this.geometry, indent + 1, "geometry ");
      _ref = this.materials;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        material = _ref[_i];
        output += getNodeInfo(material, indent + 1, "material ");
      }
      return output;
    };

    return ColladaInstanceGeometry;

  })();

  ColladaInstanceController = (function() {

    function ColladaInstanceController() {
      this.controller = null;
      this.skeletons = [];
      this.materials = [];
      this.sidChildren = [];
    }

    ColladaInstanceController.prototype.getInfo = function(indent, prefix) {
      var material, output, skeleton, _i, _j, _len, _len2, _ref, _ref2;
      output = graphNodeString(indent, prefix + "<instanceController>\n");
      output += getNodeInfo(this.controller, indent + 1, "controller ");
      _ref = this.skeletons;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        skeleton = _ref[_i];
        output += getNodeInfo(skeleton, indent + 1, "skeleton ");
      }
      _ref2 = this.materials;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        material = _ref2[_j];
        output += getNodeInfo(material, indent + 1, "material ");
      }
      return output;
    };

    return ColladaInstanceController;

  })();

  ColladaInstanceMaterial = (function() {

    function ColladaInstanceMaterial() {
      this.sid = null;
      this.symbol = null;
      this.material = null;
      this.name = null;
      this.vertexInputs = {};
      this.params = {};
    }

    ColladaInstanceMaterial.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<instanceMaterial sid='" + this.sid + "'>\n"));
      output += getNodeInfo(this.material, indent + 1, "material ");
      return output;
    };

    return ColladaInstanceMaterial;

  })();

  ColladaInstanceLight = (function() {

    function ColladaInstanceLight() {
      this.sid = null;
      this.light = null;
      this.name = null;
      this.sidChildren = [];
    }

    ColladaInstanceLight.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + "<instanceLight>\n");
      output += getNodeInfo(this.light, indent + 1, "light ");
      return output;
    };

    return ColladaInstanceLight;

  })();

  ColladaInstanceCamera = (function() {

    function ColladaInstanceCamera() {
      this.sid = null;
      this.camera = null;
      this.name = null;
      this.sidChildren = [];
    }

    ColladaInstanceCamera.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + "<instanceCamera>\n");
      output += getNodeInfo(this.light, indent + 1, "camera ");
      return output;
    };

    return ColladaInstanceCamera;

  })();

  ColladaImage = (function() {

    function ColladaImage() {
      this.id = null;
      this.initFrom = null;
    }

    ColladaImage.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<image id='" + this.id + "'>\n"));
      output += getNodeInfo(this.initFrom, indent + 1, "initFrom ");
      return output;
    };

    return ColladaImage;

  })();

  ColladaEffect = (function() {

    function ColladaEffect() {
      this.id = null;
      this.sids = {};
      this.technique = null;
    }

    ColladaEffect.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<effect id='" + this.id + "'>\n"));
      output += getNodeInfo(this.technique, indent + 1, "technique ");
      return output;
    };

    return ColladaEffect;

  })();

  ColladaEffectTechnique = (function() {

    function ColladaEffectTechnique() {
      this.sid = null;
      this.sids = {};
      this.fxScope = null;
      this.shading = null;
      this.emission = null;
      this.ambient = null;
      this.diffuse = null;
      this.specular = null;
      this.shininess = null;
      this.reflective = null;
      this.transparent = null;
      this.bump = null;
      this.reflectivity = null;
      this.transparency = null;
      this.index_of_refraction = null;
    }

    ColladaEffectTechnique.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<technique sid='" + this.sid + "'>\n"));
      return output;
    };

    return ColladaEffectTechnique;

  })();

  ColladaEffectSurface = (function() {

    function ColladaEffectSurface() {
      this.sid = null;
      this.fxScope = null;
      this.type = null;
      this.initFrom = null;
      this.format = null;
      this.size = null;
      this.viewportRatio = null;
      this.mipLevels = null;
      this.mipmapGenerate = null;
    }

    ColladaEffectSurface.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<surface sid='" + this.sid + "'>\n"));
      output += getNodeInfo(this.initFrom, indent + 1, "initFrom ");
      return output;
    };

    return ColladaEffectSurface;

  })();

  ColladaEffectSampler = (function() {

    function ColladaEffectSampler() {
      this.sid = null;
      this.fxScope = null;
      this.surface = null;
      this.image = null;
      this.wrapS = null;
      this.wrapT = null;
      this.minfilter = null;
      this.magfilter = null;
      this.borderColor = null;
      this.mipmapMaxLevel = null;
      this.mipmapBias = null;
    }

    ColladaEffectSampler.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<sampler sid='" + this.sid + "'>\n"));
      output += getNodeInfo(this.image, indent + 1, "image ");
      output += getNodeInfo(this.surface, indent + 1, "surface ");
      return output;
    };

    return ColladaEffectSampler;

  })();

  ColladaColorOrTexture = (function() {

    function ColladaColorOrTexture() {
      this.color = null;
      this.textureSampler = null;
      this.texcoord = null;
      this.opaque = null;
      this.bumptype = null;
    }

    return ColladaColorOrTexture;

  })();

  ColladaMaterial = (function() {

    function ColladaMaterial() {
      this.id = null;
      this.name = null;
      this.effect = null;
    }

    ColladaMaterial.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<material id='" + this.id + "' name='" + this.name + "'>\n"));
      output += getNodeInfo(this.effect, indent + 1, "effect ");
      return output;
    };

    return ColladaMaterial;

  })();

  ColladaGeometry = (function() {

    function ColladaGeometry() {
      this.id = null;
      this.name = null;
      this.sources = [];
      this.vertices = null;
      this.triangles = [];
    }

    ColladaGeometry.prototype.getInfo = function(indent, prefix) {
      var output, source, tri, _i, _j, _len, _len2, _ref, _ref2;
      output = graphNodeString(indent, prefix + ("<geometry id='" + this.id + "' name='" + this.name + "'>\n"));
      _ref = this.sources;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        source = _ref[_i];
        output += getNodeInfo(source, indent + 1, "source ");
      }
      output += getNodeInfo(this.vertices, indent + 1, "vertices ");
      _ref2 = this.triangles;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        tri = _ref2[_j];
        output += getNodeInfo(tri, indent + 1, "triangles ");
      }
      return output;
    };

    return ColladaGeometry;

  })();

  ColladaSource = (function() {

    function ColladaSource() {
      this.id = null;
      this.name = null;
      this.sourceId = null;
      this.count = null;
      this.stride = null;
      this.data = null;
      this.params = {};
    }

    ColladaSource.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<source id='" + this.id + "' name='" + this.name + "'>\n"));
      output += getNodeInfo(this.sourceId, indent + 1, "sourceId ");
      return output;
    };

    return ColladaSource;

  })();

  ColladaVertices = (function() {

    function ColladaVertices() {
      this.id = null;
      this.name = null;
      this.inputs = [];
    }

    ColladaVertices.prototype.getInfo = function(indent, prefix) {
      var input, output, _i, _len, _ref;
      output = graphNodeString(indent, prefix + ("<vertices id='" + this.id + "' name='" + this.name + "'>\n"));
      _ref = this.inputs;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        input = _ref[_i];
        output += getNodeInfo(input, indent + 1, "input ");
      }
      return output;
    };

    return ColladaVertices;

  })();

  ColladaTriangles = (function() {

    function ColladaTriangles() {
      this.name = null;
      this.count = null;
      this.material = null;
      this.inputs = [];
      this.indices = null;
    }

    ColladaTriangles.prototype.getInfo = function(indent, prefix) {
      var input, output, _i, _len, _ref;
      output = graphNodeString(indent, prefix + ("<triangles name='" + this.name + "'>\n"));
      output += getNodeInfo(this.material, indent + 1, "material ");
      _ref = this.inputs;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        input = _ref[_i];
        output += getNodeInfo(input, indent + 1, "input ");
      }
      return output;
    };

    return ColladaTriangles;

  })();

  ColladaInput = (function() {

    function ColladaInput() {
      this.semantic = null;
      this.source = null;
      this.offset = null;
      this.set = null;
    }

    ColladaInput.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<input semantic=" + this.semantic + ">\n"));
      output += getNodeInfo(this.source, indent + 1, "source ");
      return output;
    };

    return ColladaInput;

  })();

  ColladaController = (function() {

    function ColladaController() {
      this.id = null;
      this.name = null;
      this.skin = null;
      this.morph = null;
    }

    ColladaController.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + ("<controller id='" + this.id + "', name='" + this.name + "'>\n"));
      output += getNodeInfo(this.skin, indent + 1, "skin ");
      output += getNodeInfo(this.morph, indent + 1, "morph ");
      return output;
    };

    return ColladaController;

  })();

  ColladaSkin = (function() {

    function ColladaSkin() {
      this.source = null;
      this.bindShapeMatrix = null;
      this.sources = [];
      this.joints = null;
      this.vertexWeights = null;
    }

    ColladaSkin.prototype.getInfo = function(indent, prefix) {
      var output, source, _i, _len, _ref;
      output = graphNodeString(indent, prefix + ("<skin source='" + this.source + "'>\n"));
      output += getNodeInfo(this.bindShapeMatrix, indent + 1, "bind_shape_matrix ");
      _ref = this.sources;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        source = _ref[_i];
        output += getNodeInfo(source, indent + 1, "source ");
      }
      output += getNodeInfo(this.joints, indent + 1, "joints ");
      output += getNodeInfo(this.vertexWeights, indent + 1, "vertex_weights ");
      return output;
    };

    return ColladaSkin;

  })();

  ColladaJoints = (function() {

    function ColladaJoints() {
      this.joints = null;
      this.invBindMatrices = null;
    }

    ColladaJoints.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + "<joints>\n");
      output += getNodeInfo(this.joints, indent + 1, "joints ");
      output += getNodeInfo(this.invBindMatrices, indent + 1, "invBindMatrices ");
      return output;
    };

    return ColladaJoints;

  })();

  ColladaVertexWeights = (function() {

    function ColladaVertexWeights() {
      this.inputs = [];
      this.vcount = null;
      this.v = null;
      this.joints = null;
      this.weights = null;
    }

    ColladaVertexWeights.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + "<vertex_weights>\n");
      output += getNodeInfo(this.joints, indent + 1, "joints ");
      output += getNodeInfo(this.weights, indent + 1, "weights ");
      return output;
    };

    return ColladaVertexWeights;

  })();

  ColladaAnimation = (function() {

    function ColladaAnimation() {
      this.id = null;
      this.name = null;
      this.parent = null;
      this.rootId = null;
      this.rootName = null;
      this.animations = [];
      this.sources = [];
      this.samplers = [];
      this.channels = [];
    }

    ColladaAnimation.prototype.getInfo = function(indent, prefix) {
      var animation, channel, output, sampler, source, _i, _j, _k, _l, _len, _len2, _len3, _len4, _ref, _ref2, _ref3, _ref4;
      output = graphNodeString(indent, prefix + ("<animation id='" + this.id + "', name='" + name + "'>\n"));
      _ref = this.animations;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        animation = _ref[_i];
        output += getNodeInfo(animation, indent + 1, "animation ");
      }
      _ref2 = this.sources;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        source = _ref2[_j];
        output += getNodeInfo(source, indent + 1, "source ");
      }
      _ref3 = this.samplers;
      for (_k = 0, _len3 = _ref3.length; _k < _len3; _k++) {
        sampler = _ref3[_k];
        output += getNodeInfo(sampler, indent + 1, "sampler ");
      }
      _ref4 = this.channels;
      for (_l = 0, _len4 = _ref4.length; _l < _len4; _l++) {
        channel = _ref4[_l];
        output += getNodeInfo(channel, indent + 1, "channel ");
      }
      return output;
    };

    return ColladaAnimation;

  })();

  ColladaSampler = (function() {

    function ColladaSampler() {
      this.id = null;
      this.input = null;
      this.outputs = [];
      this.inTangents = [];
      this.outTangents = [];
      this.interpolation = null;
    }

    ColladaSampler.prototype.getInfo = function(indent, prefix) {
      var o, output, t, _i, _j, _k, _len, _len2, _len3, _ref, _ref2, _ref3;
      output = graphNodeString(indent, prefix + ("<sampler id='" + this.id + "'>\n"));
      output += getNodeInfo(this.input, indent + 1, "input ");
      _ref = this.outputs;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        o = _ref[_i];
        output += getNodeInfo(o, indent + 1, "output ");
      }
      _ref2 = this.inTangents;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        t = _ref2[_j];
        output += getNodeInfo(t, indent + 1, "inTangent ");
      }
      _ref3 = this.outTangents;
      for (_k = 0, _len3 = _ref3.length; _k < _len3; _k++) {
        t = _ref3[_k];
        output += getNodeInfo(t, indent + 1, "outTangent ");
      }
      output += getNodeInfo(this.interpolation, indent + 1, "interpolation ");
      return output;
    };

    return ColladaSampler;

  })();

  ColladaChannel = (function() {

    function ColladaChannel() {
      this.animation = null;
      this.source = null;
      this.target = null;
    }

    ColladaChannel.prototype.getInfo = function(indent, prefix) {
      var output;
      output = graphNodeString(indent, prefix + "<channel>\n");
      output += getNodeInfo(this.source, indent + 1, "source ");
      output += getNodeInfo(this.target, indent + 1, "target ");
      return output;
    };

    return ColladaChannel;

  })();

  ColladaLight = (function() {

    function ColladaLight() {
      this.id = null;
      this.name = null;
      this.type = null;
      this.color = null;
      this.params = {};
      this.sidChildren = [];
    }

    ColladaLight.prototype.getInfo = function(indent, prefix) {
      return graphNodeString(indent, prefix + "<light>\n");
    };

    return ColladaLight;

  })();

  ColladaLightParam = (function() {

    function ColladaLightParam() {
      this.sid = null;
      this.name = null;
      this.value = null;
    }

    return ColladaLightParam;

  })();

  ColladaCamera = (function() {

    function ColladaCamera() {
      this.id = null;
      this.name = null;
      this.type = null;
      this.params = {};
      this.sidChildren = [];
    }

    ColladaCamera.prototype.getInfo = function(indent, prefix) {
      return graphNodeString(indent, prefix + "<camera>\n");
    };

    return ColladaCamera;

  })();

  ColladaCameraParam = (function() {

    function ColladaCameraParam() {
      this.sid = null;
      this.name = null;
      this.value = null;
    }

    return ColladaCameraParam;

  })();

  ThreejsAnimationChannel = (function() {

    function ThreejsAnimationChannel() {
      this.inputData = null;
      this.outputData = null;
      this.offset = null;
      this.stride = null;
      this.count = null;
      this.animation = null;
    }

    return ThreejsAnimationChannel;

  })();

  ThreejsSkeletonBone = (function() {

    function ThreejsSkeletonBone() {
      this.index = null;
      this.node = null;
      this.sid = null;
      this.parent = null;
      this.isAnimated = null;
      this.matrix = new THREE.Matrix4;
      this.worldMatrix = new THREE.Matrix4;
      this.invBindMatrix = new THREE.Matrix4;
      this.skinMatrix = new THREE.Matrix4;
      this.worldMatrixDirty = true;
    }

    ThreejsSkeletonBone.prototype.getWorldMatrix = function() {
      if (this.worldMatrixDirty) {
        if (this.parent != null) {
          this.worldMatrix.multiplyMatrices(this.parent.getWorldMatrix(), this.matrix);
        } else {
          this.worldMatrix.copy(this.matrix);
        }
        this.worldMatrixDirty = false;
      }
      return this.worldMatrix;
    };

    ThreejsSkeletonBone.prototype.applyAnimation = function(frame) {
      var transform, _i, _len, _ref;
      if (this.isAnimated) {
        _ref = this.node.transformations;
        for (_i = 0, _len = _ref.length; _i < _len; _i++) {
          transform = _ref[_i];
          transform.applyAnimationKeyframe(frame);
        }
        this.node.getTransformMatrix(this.matrix);
      }
      this.worldMatrixDirty = true;
      return null;
    };

    ThreejsSkeletonBone.prototype.updateSkinMatrix = function(bindShapeMatrix) {
      var worldMatrix;
      worldMatrix = this.getWorldMatrix();
      this.skinMatrix.multiplyMatrices(worldMatrix, this.invBindMatrix);
      this.skinMatrix.multiplyMatrices(this.skinMatrix, bindShapeMatrix);
      return null;
    };

    return ThreejsSkeletonBone;

  })();

  ThreejsMaterialMap = (function() {

    function ThreejsMaterialMap() {
      this.materials = [];
      this.indices = {};
      this.needTangents = false;
    }

    return ThreejsMaterialMap;

  })();

  ColladaFile = (function() {

    function ColladaFile(loader) {
      var key, value, _ref;
      this._url = null;
      this._baseUrl = null;
      this._loader = loader;
      this._options = {};
      _ref = loader.options;
      for (key in _ref) {
        value = _ref[key];
        this._options[key] = value;
      }
      this._log = loader.log;
      this._readyCallback = null;
      this._progressCallback = null;
      this.dae = {};
      this.dae.ids = {};
      this.dae.animationTargets = [];
      this.dae.libEffects = [];
      this.dae.libMaterials = [];
      this.dae.libGeometries = [];
      this.dae.libControllers = [];
      this.dae.libLights = [];
      this.dae.libCameras = [];
      this.dae.libImages = [];
      this.dae.libVisualScenes = [];
      this.dae.libAnimations = [];
      this.dae.asset = null;
      this.dae.scene = null;
      this.threejs = {};
      this.threejs.scene = null;
      this.threejs.images = [];
      this.threejs.geometries = [];
      this.threejs.materials = [];
      this.scene = null;
    }

    ColladaFile.prototype.setUrl = function(url) {
      var parts;
      if (url != null) {
        this._url = url;
        parts = url.split("/");
        parts.pop();
        this._baseUrl = (parts.length < 1 ? "." : parts.join("/")) + "/";
      } else {
        this._url = "";
        this._baseUrl = "";
      }
    };

    ColladaFile.prototype.getLibInfo = function(lib, indent, libname) {
      var child, numElements, output, _i, _len;
      if (lib == null) return "";
      output = graphNodeString(indent, libname + (" <" + libname + ">\n"));
      numElements = 0;
      for (_i = 0, _len = lib.length; _i < _len; _i++) {
        child = lib[_i];
        output += getNodeInfo(child, indent + 1, "");
        numElements += 1;
      }
      if (numElements > 0) {
        return output;
      } else {
        return "";
      }
    };

    ColladaFile.prototype.getInfo = function(indent, prefix) {
      var output;
      output = "<collada url='" + this.url + "'>\n";
      output += getNodeInfo(this.dae.asset, indent + 1, "asset ");
      output += getNodeInfo(this.dae.scene, indent + 1, "scene ");
      output += this.getLibInfo(this.dae.libEffects, indent + 1, "library_effects");
      output += this.getLibInfo(this.dae.libMaterials, indent + 1, "library_materials");
      output += this.getLibInfo(this.dae.libGeometries, indent + 1, "library_geometries");
      output += this.getLibInfo(this.dae.libControllers, indent + 1, "library_controllers");
      output += this.getLibInfo(this.dae.libLights, indent + 1, "library_lights");
      output += this.getLibInfo(this.dae.libCameras, indent + 1, "library_cameras");
      output += this.getLibInfo(this.dae.libImages, indent + 1, "library_images");
      output += this.getLibInfo(this.dae.libVisualScenes, indent + 1, "library_visual_scenes");
      output += this.getLibInfo(this.dae.libAnimations, indent + 1, "library_animations");
      return output;
    };

    ColladaFile.prototype._reportUnexpectedChild = function(parent, child) {
      this._log("Skipped unknown <" + parent.nodeName + "> child <" + child.nodeName + ">.", ColladaLoader2.messageWarning);
    };

    ColladaFile.prototype._reportUnhandledExtra = function(parent, child) {
      this._log("Skipped element <" + parent.nodeName + ">/<" + child.nodeName + ">. Element is legal, but not handled by this loader.", ColladaLoader2.messageWarning);
    };

    ColladaFile.prototype._getAttributeAsFloat = function(el, name, defaultValue) {
      var data;
      data = el.getAttribute(name);
      if (data != null) {
        return parseFloat(data);
      } else {
        return defaultValue;
      }
    };

    ColladaFile.prototype._getAttributeAsInt = function(el, name, defaultValue) {
      var data;
      data = el.getAttribute(name);
      if (data != null) {
        return parseInt(data, 10);
      } else {
        return defaultValue;
      }
    };

    ColladaFile.prototype._addUrlTarget = function(object, lib, needsId) {
      var id;
      if (lib != null) lib.push(object);
      id = object.id;
      if (!(id != null)) {
        if (needsId) this._log("Object has no ID.", ColladaLoader2.messageError);
        return;
      }
      if (this.dae.ids[id] != null) {
        this._log("There is already an object with ID " + id + ".", ColladaLoader2.messageError);
        return;
      }
      this.dae.ids[id] = object;
    };

    ColladaFile.prototype._resolveUrlLink = function(link) {
      link.object = this.dae.ids[link.url];
      if (!(link.object != null)) {
        this._log("Could not resolve URL #" + link.url, ColladaLoader2.messageError);
        return false;
      }
      return true;
    };

    ColladaFile.prototype._addFxTarget = function(object, scope) {
      var sid;
      sid = object.sid;
      if (!(sid != null)) {
        this._log("Cannot add a FX target: object has no SID.", ColladaLoader2.messageError);
        return;
      }
      if (scope.sids[sid] != null) {
        this._log("There is already an FX target with SID " + sid + ".", ColladaLoader2.messageError);
        return;
      }
      object.fxScope = scope;
      scope.sids[sid] = object;
    };

    ColladaFile.prototype._resolveFxLink = function(link) {
      var scope;
      scope = link.scope;
      while (!(link.object != null) && (scope != null)) {
        link.object = scope.sids[link.url];
        scope = scope.fxScope;
      }
      if (!(link.object != null)) {
        this._log("Could not resolve FX parameter #" + link.url, ColladaLoader2.messageError);
        return false;
      }
      return true;
    };

    ColladaFile.prototype._addSidTarget = function(object, parent) {
      if (!(parent.sidChildren != null)) parent.sidChildren = [];
      parent.sidChildren.push(object);
    };

    ColladaFile.prototype._findSidTarget = function(root, sidString) {
      var childObject, front, parentObject, queue, sid, sidChild, sids, _i, _j, _len, _len2, _ref;
      sids = sidString.split("/");
      parentObject = root;
      childObject = null;
      for (_i = 0, _len = sids.length; _i < _len; _i++) {
        sid = sids[_i];
        queue = [parentObject];
        while (queue.length !== 0) {
          front = queue.shift();
          if (front.sid === sid) {
            childObject = front;
            break;
          }
          if (front.sidChildren != null) {
            _ref = front.sidChildren;
            for (_j = 0, _len2 = _ref.length; _j < _len2; _j++) {
              sidChild = _ref[_j];
              queue.push(sidChild);
            }
          }
        }
        if (!(childObject != null)) return null;
        parentObject = childObject;
      }
      return childObject;
    };

    ColladaFile.prototype._resolveSidLink = function(link) {
      var baseObject, childObject, front, parentObject, queue, sid, sidChild, _i, _j, _len, _len2, _ref, _ref2;
      baseObject = this.dae.ids[link.id];
      if (!(baseObject != null)) {
        this._log("Could not resolve SID #" + link.url + ", missing base ID " + link.id, ColladaLoader2.messageError);
        return false;
      }
      parentObject = baseObject;
      childObject = null;
      _ref = link.sids;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        sid = _ref[_i];
        queue = [parentObject];
        while (queue.length !== 0) {
          front = queue.shift();
          if (front.sid === sid) {
            childObject = front;
            break;
          }
          if (front.sidChildren != null) {
            _ref2 = front.sidChildren;
            for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
              sidChild = _ref2[_j];
              queue.push(sidChild);
            }
          }
        }
        if (!(childObject != null)) {
          this._log("Could not resolve SID #" + link.url + ", missing SID part " + sid, ColladaLoader2.messageError);
          return false;
        }
        parentObject = childObject;
      }
      link.object = childObject;
      return true;
    };

    ColladaFile.prototype._getLinkTarget = function(link, type) {
      if (!(link != null)) return null;
      if (!(link.object != null)) {
        if (link instanceof ColladaUrlLink) {
          this._resolveUrlLink(link);
        } else if (link instanceof ColladaSidLink) {
          this._resolveSidLink(link);
        } else if (link instanceof ColladaFxLink) {
          this._resolveFxLink(link);
        } else {
          this._log("Trying to resolve an object that is not a link", ColladaLoader2.messageError);
        }
      }
      if ((type != null) && (link.object != null) && !(link.object instanceof type)) {
        this._log("Link " + link.url + " does not link to a " + type.name, ColladaLoader2.messageError);
      }
      return link.object;
    };

    ColladaFile.prototype._parseXml = function(doc) {
      var colladaElement, _ref;
      colladaElement = doc.childNodes[0];
      if ((colladaElement != null ? (_ref = colladaElement.nodeName) != null ? _ref.toUpperCase() : void 0 : void 0) === "COLLADA") {
        this._parseCollada(colladaElement);
      } else {
        this._log("Can not parse document, top level element is not <COLLADA>.", ColladaLoader2.messageError);
      }
    };

    ColladaFile.prototype._parseCollada = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "asset":
              this._parseAsset(child);
              break;
            case "scene":
              this._parseScene(child);
              break;
            case "library_effects":
              this._parseLibEffect(child);
              break;
            case "library_materials":
              this._parseLibMaterial(child);
              break;
            case "library_geometries":
              this._parseLibGeometry(child);
              break;
            case "library_images":
              this._parseLibImage(child);
              break;
            case "library_visual_scenes":
              this._parseLibVisualScene(child);
              break;
            case "library_controllers":
              this._parseLibController(child);
              break;
            case "library_animations":
              this._parseLibAnimation(child);
              break;
            case "library_lights":
              this._parseLibLight(child);
              break;
            case "library_cameras":
              this._parseLibCamera(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseAsset = function(el) {
      var child, _i, _len, _ref;
      if (!this.dae.asset) this.dae.asset = new ColladaAsset();
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "unit":
              this.dae.asset.unit = this._getAttributeAsFloat(child, "meter");
              break;
            case "up_axis":
              this.dae.asset.upAxis = child.textContent.toUpperCase().charAt(0);
              break;
            case "contributor":
            case "created":
            case "modified":
            case "revision":
            case "title":
            case "subject":
            case "keywords":
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseScene = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "instance_visual_scene":
              this.dae.scene = new ColladaUrlLink(child.getAttribute("url"));
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLibVisualScene = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "visual_scene":
              this._parseVisualScene(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseVisualScene = function(el) {
      var child, scene, _i, _len, _ref;
      scene = new ColladaVisualScene;
      scene.id = el.getAttribute("id");
      this._addUrlTarget(scene, this.dae.libVisualScenes, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "node":
              this._parseSceneNode(scene, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseSceneNode = function(parent, el) {
      var child, node, _i, _len, _ref;
      node = new ColladaVisualSceneNode;
      node.id = el.getAttribute("id");
      node.sid = el.getAttribute("sid");
      node.name = el.getAttribute("name");
      node.type = el.getAttribute("type");
      node.layer = el.getAttribute("layer");
      node.parent = parent;
      parent.children.push(node);
      this._addUrlTarget(node, null, false);
      this._addSidTarget(node, parent);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "instance_geometry":
              this._parseInstanceGeometry(node, child);
              break;
            case "instance_controller":
              this._parseInstanceController(node, child);
              break;
            case "instance_light":
              this._parseInstanceLight(node, child);
              break;
            case "instance_camera":
              this._parseInstanceCamera(node, child);
              break;
            case "matrix":
            case "rotate":
            case "translate":
            case "scale":
              this._parseTransformElement(node, child);
              break;
            case "node":
              this._parseSceneNode(node, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseInstanceGeometry = function(parent, el) {
      var child, geometry, _i, _len, _ref;
      geometry = new ColladaInstanceGeometry();
      geometry.geometry = new ColladaUrlLink(el.getAttribute("url"));
      geometry.sid = el.getAttribute("sid");
      parent.geometries.push(geometry);
      this._addSidTarget(geometry, parent);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "bind_material":
              this._parseBindMaterial(geometry, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseInstanceController = function(parent, el) {
      var child, controller, _i, _len, _ref;
      controller = new ColladaInstanceController();
      controller.controller = new ColladaUrlLink(el.getAttribute("url"));
      controller.sid = el.getAttribute("sid");
      controller.name = el.getAttribute("name");
      parent.controllers.push(controller);
      this._addSidTarget(controller, parent);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "skeleton":
              controller.skeletons.push(new ColladaUrlLink(child.textContent));
              break;
            case "bind_material":
              this._parseBindMaterial(controller, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseBindMaterial = function(parent, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "technique_common":
              this._parseBindMaterialTechnique(parent, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseBindMaterialTechnique = function(parent, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "instance_material":
              this._parseInstanceMaterial(parent, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseInstanceMaterial = function(parent, el) {
      var child, inputSemantic, inputSet, material, semantic, target, _i, _len, _ref;
      material = new ColladaInstanceMaterial;
      material.symbol = el.getAttribute("symbol");
      material.material = new ColladaUrlLink(el.getAttribute("target"));
      parent.materials.push(material);
      this._addSidTarget(material, parent);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "bind_vertex_input":
              semantic = child.getAttribute("semantic");
              inputSemantic = child.getAttribute("input_semantic");
              inputSet = child.getAttribute("input_set");
              if (inputSet != null) inputSet = parseInt(inputSet, 10);
              material.vertexInputs[semantic] = {
                inputSemantic: inputSemantic,
                inputSet: inputSet
              };
              break;
            case "bind":
              semantic = child.getAttribute("semantic");
              target = new ColladaSidLink(null, child.getAttribute("target"));
              material.params[semantic] = {
                target: target
              };
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseTransformElement = function(parent, el) {
      var expectedDataLength, transform;
      transform = new ColladaNodeTransform;
      transform.sid = el.getAttribute("sid");
      transform.type = el.nodeName;
      transform.node = parent;
      parent.transformations.push(transform);
      this._addSidTarget(transform, parent);
      this.dae.animationTargets.push(transform);
      transform.data = _strToFloats(el.textContent);
      expectedDataLength = 0;
      switch (transform.type) {
        case "matrix":
          expectedDataLength = 16;
          break;
        case "rotate":
          expectedDataLength = 4;
          break;
        case "translate":
          expectedDataLength = 3;
          break;
        case "scale":
          expectedDataLength = 3;
          break;
        case "skew":
          expectedDataLength = 7;
          break;
        case "lookat":
          expectedDataLength = 9;
          break;
        default:
          this._log("Unknown transformation type " + transform.type + ".", ColladaLoader2.messageError);
      }
      if (transform.data.length !== expectedDataLength) {
        this._log("Wrong number of elements for transformation type '" + transform.type + "': expected " + expectedDataLength + ", found " + transform.data.length, ColladaLoader2.messageError);
      }
    };

    ColladaFile.prototype._parseInstanceLight = function(parent, el) {
      var child, light, _i, _len, _ref;
      light = new ColladaInstanceLight();
      light.light = new ColladaUrlLink(el.getAttribute("url"));
      light.sid = el.getAttribute("sid");
      light.name = el.getAttribute("name");
      parent.lights.push(light);
      this._addSidTarget(light, parent);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseInstanceCamera = function(parent, el) {
      var camera, child, _i, _len, _ref;
      camera = new ColladaInstanceCamera();
      camera.camera = new ColladaUrlLink(el.getAttribute("url"));
      camera.sid = el.getAttribute("sid");
      camera.name = el.getAttribute("name");
      parent.cameras.push(camera);
      this._addSidTarget(camera, parent);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLibEffect = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "effect":
              this._parseEffect(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffect = function(el) {
      var child, effect, _i, _len, _ref;
      effect = new ColladaEffect;
      effect.id = el.getAttribute("id");
      this._addUrlTarget(effect, this.dae.libEffects, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "profile_COMMON":
              this._parseEffectProfileCommon(effect, child);
              break;
            case "profile":
              this._log("Skipped non-common effect profile for effect " + effect.id + ".", ColladaLoader2.messageWarning);
              break;
            case "extra":
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffectProfileCommon = function(effect, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "newparam":
              this._parseEffectNewparam(effect, child);
              break;
            case "technique":
              this._parseEffectTechnique(effect, child);
              break;
            case "extra":
              this._parseTechniqueExtra(effect.technique, "COMMON", child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffectNewparam = function(scope, el) {
      var child, sid, _i, _len, _ref;
      sid = el.getAttribute("sid");
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "surface":
              this._parseEffectSurface(scope, sid, child);
              break;
            case "sampler2D":
              this._parseEffectSampler(scope, sid, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffectSurface = function(scope, sid, el) {
      var child, surface, _i, _len, _ref;
      surface = new ColladaEffectSurface;
      surface.type = el.getAttribute("type");
      surface.sid = sid;
      this._addFxTarget(surface, scope);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "init_from":
              surface.initFrom = new ColladaUrlLink(child.textContent);
              break;
            case "format":
              surface.format = child.textContent;
              break;
            case "size":
              surface.size = _strToFloats(child.textContent);
              break;
            case "viewport_ratio":
              surface.viewportRatio = _strToFloats(child.textContent);
              break;
            case "mip_levels":
              surface.mipLevels = parseInt(child.textContent, 10);
              break;
            case "mipmap_generate":
              surface.mipmapGenerate = child.textContent;
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffectSampler = function(scope, sid, el) {
      var child, sampler, _i, _len, _ref;
      sampler = new ColladaEffectSampler;
      sampler.sid = sid;
      this._addFxTarget(sampler, scope);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "source":
              sampler.surface = new ColladaFxLink(child.textContent, scope);
              break;
            case "instance_image":
              sampler.image = new ColladaUrlLink(child.getAttribute("url"));
              break;
            case "wrap_s":
              sampler.wrapS = child.textContent;
              break;
            case "wrap_t":
              sampler.wrapT = child.textContent;
              break;
            case "minfilter":
              sampler.minfilter = child.textContent;
              break;
            case "magfilter":
              sampler.magfilter = child.textContent;
              break;
            case "border_color":
              sampler.borderColor = _strToFloats(child.textContent);
              break;
            case "mipmap_maxlevel":
              sampler.mipmapMaxLevel = parseInt(child.textContent, 10);
              break;
            case "mipmap_bias":
              sampler.mipmapBias = parseFloat(child.textContent);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffectTechnique = function(effect, el) {
      var child, technique, _i, _len, _ref;
      technique = new ColladaEffectTechnique;
      technique.sid = el.getAttribute("sid");
      this._addFxTarget(technique, effect);
      effect.technique = technique;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "blinn":
            case "phong":
            case "lambert":
            case "constant":
              technique.shading = child.nodeName;
              this._parseTechniqueParam(technique, "COMMON", child);
              break;
            case "extra":
              this._parseTechniqueExtra(technique, "COMMON", child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseTechniqueParam = function(technique, profile, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "newparam":
              this._parseEffectNewparam(technique, child);
              break;
            case "emission":
            case "ambient":
            case "diffuse":
            case "specular":
            case "reflective":
              this._parseEffectColorOrTexture(technique, child);
              break;
            case "shininess":
            case "reflectivity":
            case "transparency":
            case "index_of_refraction":
              technique[child.nodeName] = parseFloat(child.childNodes[1].textContent);
              break;
            case "transparent":
              this._parseEffectColorOrTexture(technique, child);
              technique.transparent.opaque = child.getAttribute("opaque");
              break;
            case "bump":
              this._parseEffectColorOrTexture(technique, child);
              technique.bump.bumptype = child.getAttribute("bumptype");
              break;
            case "double_sided":
              technique.doubleSided = parseInt(child.textContent, 10) === 1 ? true : false;
              break;
            default:
              if (profile === "COMMON") this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseTechniqueExtra = function(technique, profile, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "technique":
              profile = child.getAttribute("profile");
              this._parseTechniqueParam(technique, profile, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseEffectColorOrTexture = function(technique, el) {
      var child, colorOrTexture, name, texture, _i, _len, _ref;
      name = el.nodeName;
      colorOrTexture = technique[name];
      if (!(colorOrTexture != null)) {
        colorOrTexture = new ColladaColorOrTexture();
        technique[name] = colorOrTexture;
      }
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "color":
              colorOrTexture.color = _strToColor(child.textContent);
              break;
            case "texture":
              texture = child.getAttribute("texture");
              colorOrTexture.textureSampler = new ColladaFxLink(texture, technique);
              colorOrTexture.texcoord = child.getAttribute("texcoord");
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLibMaterial = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "material":
              this._parseMaterial(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseMaterial = function(el) {
      var child, material, _i, _len, _ref;
      material = new ColladaMaterial;
      material.id = el.getAttribute("id");
      material.name = el.getAttribute("name");
      this._addUrlTarget(material, this.dae.libMaterials, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "instance_effect":
              material.effect = new ColladaUrlLink(child.getAttribute("url"));
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLibGeometry = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "geometry":
              this._parseGeometry(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseGeometry = function(el) {
      var child, geometry, _i, _len, _ref;
      geometry = new ColladaGeometry();
      geometry.id = el.getAttribute("id");
      geometry.name = el.getAttribute("name");
      this._addUrlTarget(geometry, this.dae.libGeometries, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "mesh":
              this._parseMesh(geometry, child);
              break;
            case "convex_mesh":
            case "spline":
              this._log("Geometry type " + child.nodeName + " not supported.", ColladaLoader2.messageError);
              break;
            case "extra":
              this._parseGeometryExtra(geometry, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseMesh = function(geometry, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "source":
              this._parseSource(geometry, child);
              break;
            case "vertices":
              this._parseVertices(geometry, child);
              break;
            case "triangles":
            case "polylist":
            case "polygons":
              this._parseTriangles(geometry, child);
              break;
            case "lines":
            case "linestrips":
            case "trifans":
            case "tristrips":
              this._log("Geometry primitive type " + child.nodeName + " not supported.", ColladaLoader2.messageError);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseGeometryExtra = function(geometry, el) {
      var child, profile, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "technique":
              profile = child.getAttribute("profile");
              this._parseGeometryExtraTechnique(geometry, profile, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseGeometryExtraTechnique = function(geometry, profile, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "double_sided":
              geometry.doubleSided = el.textContent === "1";
          }
        }
      }
    };

    ColladaFile.prototype._parseSource = function(parent, el) {
      var child, source, _i, _len, _ref;
      source = new ColladaSource;
      source.id = el.getAttribute("id");
      source.name = el.getAttribute("name");
      this._addUrlTarget(source, parent.sources, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "bool_array":
              source.sourceId = child.getAttribute("id");
              source.data = _strToBools(child.textContent);
              break;
            case "float_array":
              source.sourceId = child.getAttribute("id");
              source.data = _strToFloats(child.textContent);
              break;
            case "int_array":
              source.sourceId = child.getAttribute("id");
              source.data = _strToInts(child.textContent);
              break;
            case "IDREF_array":
            case "Name_array":
              source.sourceId = child.getAttribute("id");
              source.data = _strToStrings(child.textContent);
              break;
            case "technique_common":
              this._parseSourceTechniqueCommon(source, child);
              break;
            case "technique":
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseVertices = function(geometry, el) {
      var child, vertices, _i, _len, _ref;
      vertices = new ColladaVertices;
      vertices.id = el.getAttribute("id");
      vertices.name = el.getAttribute("name");
      this._addUrlTarget(vertices, null, true);
      geometry.vertices = vertices;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "input":
              vertices.inputs.push(this._parseInput(child));
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseTriangles = function(geometry, el) {
      var child, triangles, _i, _len, _ref;
      triangles = new ColladaTriangles;
      triangles.name = el.getAttribute("name");
      triangles.material = el.getAttribute("material");
      triangles.count = this._getAttributeAsInt(el, "count");
      triangles.type = el.nodeName;
      geometry.triangles.push(triangles);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "input":
              triangles.inputs.push(this._parseInput(child));
              break;
            case "vcount":
              triangles.vcount = _strToInts(child.textContent);
              break;
            case "p":
              triangles.indices = _strToInts(child.textContent);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
      return triangles;
    };

    ColladaFile.prototype._parseSourceTechniqueCommon = function(source, el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "accessor":
              this._parseAccessor(source, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseAccessor = function(source, el) {
      var child, name, sourceId, type, _i, _len, _ref;
      sourceId = el.getAttribute("source");
      source.count = el.getAttribute("count");
      source.stride = this._getAttributeAsInt(el, "stride", 1);
      if (sourceId !== "#" + source.sourceId) {
        this._log("Non-local sources not supported, source data will be empty", ColladaLoader2.messageError);
      }
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "param":
              name = child.getAttribute("name");
              type = child.getAttribute("type");
              source.params[name] = type;
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseInput = function(el) {
      var input;
      input = new ColladaInput;
      input.semantic = el.getAttribute("semantic");
      input.source = new ColladaUrlLink(el.getAttribute("source"));
      input.offset = this._getAttributeAsInt(el, "offset");
      input.set = this._getAttributeAsInt(el, "set");
      return input;
    };

    ColladaFile.prototype._parseLibImage = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "image":
              this._parseImage(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseImage = function(el) {
      var child, image, _i, _len, _ref;
      image = new ColladaImage;
      image.id = el.getAttribute("id");
      this._addUrlTarget(image, this.dae.libImages, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "init_from":
              image.initFrom = child.textContent;
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLibController = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "controller":
              this._parseController(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseController = function(el) {
      var child, controller, _i, _len, _ref;
      controller = new ColladaController;
      controller.id = el.getAttribute("id");
      controller.name = el.getAttribute("name");
      this._addUrlTarget(controller, this.dae.libControllers, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "skin":
              this._parseSkin(controller, child);
              break;
            case "morph":
              this._parseMorph(controller, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseMorph = function(parent, el) {
      this._log("Morph controllers not implemented", ColladaLoader2.messageError);
    };

    ColladaFile.prototype._parseSkin = function(parent, el) {
      var child, skin, _i, _len, _ref;
      skin = new ColladaSkin;
      skin.source = new ColladaUrlLink(el.getAttribute("source"));
      if ((parent.skin != null) || (parent.morph != null)) {
        this._log("Controller already has a skin or morph", ColladaLoader2.messageError);
      }
      parent.skin = skin;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "bind_shape_matrix":
              this._parseBindShapeMatrix(skin, child);
              break;
            case "source":
              this._parseSource(skin, child);
              break;
            case "joints":
              this._parseJoints(skin, child);
              break;
            case "vertex_weights":
              this._parseVertexWeights(skin, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseBindShapeMatrix = function(parent, el) {
      parent.bindShapeMatrix = _strToFloats(el.textContent);
    };

    ColladaFile.prototype._parseJoints = function(parent, el) {
      var child, input, inputs, joints, _i, _j, _len, _len2, _ref;
      joints = new ColladaJoints;
      if (parent.joints != null) {
        this._log("Skin already has a joints array", ColladaLoader2.messageError);
      }
      parent.joints = joints;
      inputs = [];
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "input":
              inputs.push(this._parseInput(child));
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
      for (_j = 0, _len2 = inputs.length; _j < _len2; _j++) {
        input = inputs[_j];
        switch (input.semantic) {
          case "JOINT":
            joints.joints = input;
            break;
          case "INV_BIND_MATRIX":
            joints.invBindMatrices = input;
            break;
          default:
            this._log("Unknown joints input semantic " + input.semantic, ColladaLoader2.messageError);
        }
      }
    };

    ColladaFile.prototype._parseVertexWeights = function(parent, el) {
      var child, input, inputs, weights, _i, _j, _len, _len2, _ref;
      weights = new ColladaVertexWeights;
      weights.count = parseInt(el.getAttribute("count"), 10);
      if (parent.vertexWeights != null) {
        this._log("Skin already has a vertex weight array", ColladaLoader2.messageError);
      }
      parent.vertexWeights = weights;
      inputs = [];
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "input":
              inputs.push(this._parseInput(child));
              break;
            case "vcount":
              weights.vcount = _strToInts(child.textContent);
              break;
            case "v":
              weights.v = _strToInts(child.textContent);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
      for (_j = 0, _len2 = inputs.length; _j < _len2; _j++) {
        input = inputs[_j];
        switch (input.semantic) {
          case "JOINT":
            weights.joints = input;
            break;
          case "WEIGHT":
            weights.weights = input;
            break;
          default:
            this._log("Unknown vertex weight input semantic " + input.semantic, ColladaLoader2.messageError);
        }
      }
    };

    ColladaFile.prototype._parseLibAnimation = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "animation":
              this._parseAnimation(null, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseAnimation = function(parent, el) {
      var animation, child, _i, _len, _ref;
      animation = new ColladaAnimation;
      animation.id = el.getAttribute("id");
      animation.name = el.getAttribute("name");
      animation.parent = parent;
      if (parent != null) {
        animation.rootId = parent.rootId;
        animation.rootName = parent.rootName;
      } else {
        animation.rootId = animation.id;
        animation.rootName = animation.name;
      }
      this._addUrlTarget(animation, (parent != null ? parent.animations : void 0) || this.dae.libAnimations, false);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "animation":
              this._parseAnimation(animation, child);
              break;
            case "source":
              this._parseSource(animation, child);
              break;
            case "sampler":
              this._parseSampler(animation, child);
              break;
            case "channel":
              this._parseChannel(animation, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseSampler = function(parent, el) {
      var child, input, inputs, sampler, _i, _j, _len, _len2, _ref;
      sampler = new ColladaSampler;
      sampler.id = el.getAttribute("id");
      if (sampler.id != null) this._addUrlTarget(sampler, parent.samplers, false);
      inputs = [];
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "input":
              inputs.push(this._parseInput(child));
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
      for (_j = 0, _len2 = inputs.length; _j < _len2; _j++) {
        input = inputs[_j];
        switch (input.semantic) {
          case "INPUT":
            sampler.input = input;
            break;
          case "OUTPUT":
            sampler.outputs.push(input);
            break;
          case "INTERPOLATION":
            sampler.interpolation = input;
            break;
          case "IN_TANGENT":
            sampler.inTangents.push(input);
            break;
          case "OUT_TANGENT":
            sampler.outTangents.push(input);
            break;
          default:
            this._log("Unknown sampler input semantic " + input.semantic, ColladaLoader2.messageError);
        }
      }
    };

    ColladaFile.prototype._parseChannel = function(parent, el) {
      var channel, child, _i, _len, _ref;
      channel = new ColladaChannel;
      channel.source = new ColladaUrlLink(el.getAttribute("source"));
      channel.target = new ColladaSidLink(parent.id, el.getAttribute("target"));
      parent.channels.push(channel);
      channel.animation = parent;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) this._reportUnexpectedChild(el, child);
      }
    };

    ColladaFile.prototype._parseLibLight = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "light":
              this._parseLight(child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLight = function(el) {
      var child, light, _i, _len, _ref;
      light = new ColladaLight();
      light.id = el.getAttribute("id");
      light.name = el.getAttribute("name");
      if (light.id != null) this._addUrlTarget(light, this.dae.libLights, true);
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "technique_common":
              this._parseLightTechniqueCommon(child, light);
              break;
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLightTechniqueCommon = function(el, light) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "ambient":
              this._parseLightParams(child, "COMMON", light);
              break;
            case "directional":
              this._parseLightParams(child, "COMMON", light);
              break;
            case "point":
              this._parseLightParams(child, "COMMON", light);
              break;
            case "spot":
              this._parseLightParams(child, "COMMON", light);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLightParams = function(el, profile, light) {
      var child, _i, _len, _ref;
      light.type = el.nodeName;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "color":
              this._parseLightColor(child, profile, light);
              break;
            case "constant_attenuation":
              this._parseLightParam(child, profile, light);
              break;
            case "linear_attenuation":
              this._parseLightParam(child, profile, light);
              break;
            case "quadratic_attenuation":
              this._parseLightParam(child, profile, light);
              break;
            case "falloff_angle":
              this._parseLightParam(child, profile, light);
              break;
            case "falloff_exponent":
              this._parseLightParam(child, profile, light);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseLightColor = function(el, profile, light) {
      light.color = _strToFloats(el.textContent);
    };

    ColladaFile.prototype._parseLightParam = function(el, profile, light) {
      var param;
      param = new ColladaLightParam();
      param.sid = el.getAttribute("sid");
      param.name = el.nodeName;
      light.params[param.name] = param;
      this._addSidTarget(param, light);
      param.value = parseFloat(el.textContent);
    };

    ColladaFile.prototype._parseLibCamera = function(el) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "camera":
              this._parseCamera(child);
              break;
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseCamera = function(el) {
      var camera, child, _i, _len, _ref;
      camera = new ColladaCamera;
      camera.id = el.getAttribute("id");
      if (camera.id != null) this._addUrlTarget(camera, this.dae.libCameras, true);
      camera.name = el.getAttribute("name");
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "asset":
              this._reportUnhandledExtra(el, child);
              break;
            case "optics":
              this._parseCameraOptics(child, camera);
              break;
            case "imager":
              this._reportUnhandledExtra(el, child);
              break;
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseCameraOptics = function(el, camera) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "technique_common":
              this._parseCameraTechniqueCommon(child, camera);
              break;
            case "technique":
              this._reportUnhandledExtra(el, child);
              break;
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseCameraTechniqueCommon = function(el, camera) {
      var child, _i, _len, _ref;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "orthographic":
              this._parseCameraParams(child, camera);
              break;
            case "perspective":
              this._parseCameraParams(child, camera);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseCameraParams = function(el, camera) {
      var child, _i, _len, _ref;
      camera.type = el.nodeName;
      _ref = el.childNodes;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        child = _ref[_i];
        if (child.nodeType === 1) {
          switch (child.nodeName) {
            case "xmag":
              this._parseCameraParam(child, camera);
              break;
            case "ymag":
              this._parseCameraParam(child, camera);
              break;
            case "xfov":
              this._parseCameraParam(child, camera);
              break;
            case "yfov":
              this._parseCameraParam(child, camera);
              break;
            case "aspect_ratio":
              this._parseCameraParam(child, camera);
              break;
            case "znear":
              this._parseCameraParam(child, camera);
              break;
            case "zfar":
              this._parseCameraParam(child, camera);
              break;
            case "extra":
              this._reportUnhandledExtra(el, child);
              break;
            default:
              this._reportUnexpectedChild(el, child);
          }
        }
      }
    };

    ColladaFile.prototype._parseCameraParam = function(el, camera) {
      var param;
      param = new ColladaCameraParam();
      param.sid = el.getAttribute("sid");
      param.name = el.nodeName;
      camera.params[param.name] = param;
      this._addSidTarget(param, camera);
      param.value = parseFloat(el.textContent);
    };

    ColladaFile.prototype._linkAnimations = function() {
      var animation, target, _i, _j, _len, _len2, _ref, _ref2;
      _ref = this.dae.animationTargets;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        target = _ref[_i];
        target.initAnimationTarget();
      }
      _ref2 = this.dae.libAnimations;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        animation = _ref2[_j];
        this._linkAnimationChannels(animation);
      }
    };

    ColladaFile.prototype._linkAnimationChannels = function(animation) {
      var channel, child, inputSource, output, outputSource, sampler, target, threejsChannel, _i, _j, _len, _len2, _ref, _ref2, _ref3;
      _ref = animation.channels;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        channel = _ref[_i];
        target = this._getLinkTarget(channel.target, ColladaAnimationTarget);
        if (!(target != null)) {
          this._log("Animation channel has an invalid target '" + channel.target.url + "', animation ignored", ColladaLoader2.messageWarning);
          continue;
        }
        sampler = this._getLinkTarget(channel.source, ColladaSampler);
        if (!(sampler != null)) {
          this._log("Animation channel has an invalid sampler '" + channel.source.url + "', animation ignored", ColladaLoader2.messageWarning);
          continue;
        }
        inputSource = this._getLinkTarget((_ref2 = sampler.input) != null ? _ref2.source : void 0);
        if (!(inputSource != null)) {
          this._log("Animation channel has no input data, animation ignored", ColladaLoader2.messageWarning);
          continue;
        }
        if (sampler.outputs.length === 0) {
          this._log("Animation channel has no output, animation ignored", ColladaLoader2.messageWarning);
          continue;
        }
        if (sampler.outputs.length > 1) {
          this._log("Animation channel has more than one output, using only the first output", ColladaLoader2.messageWarning);
        }
        output = sampler.outputs[0];
        outputSource = this._getLinkTarget(output != null ? output.source : void 0);
        if (!(outputSource != null)) {
          this._log("Animation channel has no output data, animation ignored", ColladaLoader2.messageWarning);
          continue;
        }
        threejsChannel = new ThreejsAnimationChannel;
        threejsChannel.outputData = outputSource.data;
        threejsChannel.inputData = inputSource.data;
        threejsChannel.stride = outputSource.stride;
        threejsChannel.animation = animation;
        if (channel.target.dotSyntax) {
          threejsChannel.semantic = channel.target.member;
          threejsChannel.count = 1;
          switch (threejsChannel.semantic) {
            case "X":
              threejsChannel.offset = 0;
              break;
            case "Y":
              threejsChannel.offset = 1;
              break;
            case "Z":
              threejsChannel.offset = 2;
              break;
            case "W":
              threejsChannel.offset = 3;
              break;
            case "R":
              threejsChannel.offset = 0;
              break;
            case "G":
              threejsChannel.offset = 1;
              break;
            case "B":
              threejsChannel.offset = 2;
              break;
            case "U":
              threejsChannel.offset = 0;
              break;
            case "V":
              threejsChannel.offset = 1;
              break;
            case "S":
              threejsChannel.offset = 0;
              break;
            case "T":
              threejsChannel.offset = 1;
              break;
            case "P":
              threejsChannel.offset = 2;
              break;
            case "Q":
              threejsChannel.offset = 3;
              break;
            case "ANGLE":
              threejsChannel.offset = 3;
              break;
            default:
              this._log("Unknown semantic for '" + targetLink.url + "', animation ignored", ColladaLoader2.messageWarning);
              continue;
          }
        } else if (channel.target.arrSyntax) {
          switch (targetLink.indices.length) {
            case 1:
              threejsChannel.offset = targetLink.indices[0];
              break;
            case 2:
              threejsChannel.offset = targetLink.indices[0] * target.animTarget.dataRows + targetLink.indices[1];
              break;
            default:
              this._log("Invalid number of indices for '" + targetLink.url + "', animation ignored", ColladaLoader2.messageWarning);
              continue;
          }
          threejsChannel.count = 1;
        } else {
          threejsChannel.offset = 0;
          threejsChannel.count = target.animTarget.dataColumns * target.animTarget.dataRows;
        }
        target.animTarget.channels.push(threejsChannel);
      }
      _ref3 = animation.animations;
      for (_j = 0, _len2 = _ref3.length; _j < _len2; _j++) {
        child = _ref3[_j];
        this._linkAnimationChannels(child);
      }
    };

    ColladaFile.prototype._createSceneGraph = function() {
      var daeChild, daeScene, threejsScene, _i, _len, _ref;
      daeScene = this._getLinkTarget(this.dae.scene, ColladaVisualScene);
      if (!(daeScene != null)) return;
      threejsScene = new THREE.Object3D();
      this.threejs.scene = threejsScene;
      _ref = daeScene.children;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        daeChild = _ref[_i];
        this._createSceneGraphNode(daeChild, threejsScene);
      }
      this.scene = threejsScene;
    };

    ColladaFile.prototype._setNodeTransformation = function(daeNode, threejsNode) {
      daeNode.getTransformMatrix(threejsNode.matrix);
      threejsNode.matrix.decompose(threejsNode.position, threejsNode.quaternion, threejsNode.scale);
      threejsNode.rotation.setFromQuaternion(threejsNode.quaternion);
    };

    ColladaFile.prototype._createSceneGraphNode = function(daeNode, threejsParent) {
      var daeCamera, daeChild, daeController, daeGeometry, daeLight, threejsCamera, threejsChild, threejsChildren, threejsLight, threejsMesh, threejsNode, _i, _j, _k, _l, _len, _len2, _len3, _len4, _len5, _len6, _m, _n, _ref, _ref2, _ref3, _ref4, _ref5;
      threejsChildren = [];
      _ref = daeNode.geometries;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        daeGeometry = _ref[_i];
        threejsMesh = this._createStaticMesh(daeGeometry);
        if (threejsMesh != null) {
          threejsMesh.name = daeNode.name != null ? daeNode.name : "";
          threejsChildren.push(threejsMesh);
        }
      }
      _ref2 = daeNode.controllers;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        daeController = _ref2[_j];
        threejsMesh = this._createAnimatedMesh(daeController);
        if (threejsMesh != null) {
          threejsMesh.name = daeNode.name != null ? daeNode.name : "";
          threejsChildren.push(threejsMesh);
        }
      }
      _ref3 = daeNode.lights;
      for (_k = 0, _len3 = _ref3.length; _k < _len3; _k++) {
        daeLight = _ref3[_k];
        threejsLight = this._createLight(daeLight);
        if (threejsLight != null) {
          threejsLight.name = daeNode.name != null ? daeNode.name : "";
          threejsChildren.push(threejsLight);
        }
      }
      _ref4 = daeNode.cameras;
      for (_l = 0, _len4 = _ref4.length; _l < _len4; _l++) {
        daeCamera = _ref4[_l];
        threejsCamera = this._createCamera(daeCamera);
        if (threejsCamera != null) {
          threejsCamera.name = daeNode.name != null ? daeNode.name : "";
          threejsChildren.push(threejsCamera);
        }
      }
      if (threejsChildren.length > 1) {
        threejsNode = new THREE.Object3D();
        for (_m = 0, _len5 = threejsChildren.length; _m < _len5; _m++) {
          threejsChild = threejsChildren[_m];
          if (threejsChild != null) threejsNode.add(threejsChild);
        }
        threejsParent.add(threejsNode);
      } else if (threejsChildren.length === 1) {
        threejsNode = threejsChildren[0];
        threejsParent.add(threejsNode);
      } else if (threejsChildren.length === 0) {
        if (daeNode.type !== "JOINT") {
          this._log("Collada node " + daeNode.name + " did not produce any threejs nodes", ColladaLoader2.messageWarning);
        }
        threejsNode = new THREE.Object3D();
        threejsParent.add(threejsNode);
      }
      this._setNodeTransformation(daeNode, threejsNode);
      _ref5 = daeNode.children;
      for (_n = 0, _len6 = _ref5.length; _n < _len6; _n++) {
        daeChild = _ref5[_n];
        this._createSceneGraphNode(daeChild, threejsNode);
      }
    };

    ColladaFile.prototype._createLight = function(daeInstanceLight) {
      var attConst, attLin, attQuad, color, colorHex, foAngle, foExp, light, _ref, _ref2, _ref3, _ref4, _ref5;
      light = this._getLinkTarget(daeInstanceLight.light, ColladaLight);
      if (!(light != null)) {
        this._log("Light instance has no light, light ignored", ColladaLoader2.messageWarning);
        return null;
      }
      color = light.color;
      colorHex = (color[0] * 255) << 16 ^ (color[1] * 255) << 8 ^ (color[2] * 255) << 0;
      attConst = (_ref = light.params["constant_attenuation"]) != null ? _ref.value : void 0;
      attLin = (_ref2 = light.params["linear_attenuation"]) != null ? _ref2.value : void 0;
      attQuad = (_ref3 = light.params["quadratic_attenuation"]) != null ? _ref3.value : void 0;
      foAngle = (_ref4 = light.params["falloff_angle"]) != null ? _ref4.value : void 0;
      foExp = (_ref5 = light.params["falloff_exponent"]) != null ? _ref5.value : void 0;
      switch (light.type) {
        case "ambient":
          light = new THREE.AmbientLight(colorHex);
          break;
        case "directional":
          light = new THREE.DirectionalLight(colorHex, 1);
          break;
        case "point":
          light = new THREE.PointLight(colorHex, attConst, attLin);
          break;
        case "spot":
          light = new THREE.SpotLight(colorHex, attConst, attLin, foAngle, foExp);
          break;
        default:
          this._log("Unknown light type " + daeInstanceLight.type + ", light ignored.", ColladaLoader2.messageError);
      }
      return light;
    };

    ColladaFile.prototype._createCamera = function(daeInstanceCamera) {
      var aspect, camera, x_fov, x_mag, y_fov, y_mag, z_max, z_min, _ref, _ref2, _ref3, _ref4, _ref5, _ref6, _ref7;
      camera = this._getLinkTarget(daeInstanceCamera.camera, ColladaCamera);
      if (!(camera != null)) {
        this._log("Camera instance has no camera, camera ignored", ColladaLoader2.messageWarning);
        return null;
      }
      x_mag = (_ref = camera.params["xmag"]) != null ? _ref.value : void 0;
      y_mag = (_ref2 = camera.params["ymag"]) != null ? _ref2.value : void 0;
      x_fov = (_ref3 = camera.params["xfov"]) != null ? _ref3.value : void 0;
      y_fov = (_ref4 = camera.params["yfov"]) != null ? _ref4.value : void 0;
      aspect = (_ref5 = camera.params["aspect_ratio"]) != null ? _ref5.value : void 0;
      z_min = (_ref6 = camera.params["znear"]) != null ? _ref6.value : void 0;
      z_max = (_ref7 = camera.params["zfar"]) != null ? _ref7.value : void 0;
      switch (camera.type) {
        case "orthographic":
          if ((x_mag != null) && (y_mag != null)) {
            aspect = x_mag / y_mag;
          } else if ((y_mag != null) && (aspect != null)) {
            x_mag = y_mag * aspect;
          } else if ((x_mag != null) && (aspect != null)) {
            y_mag = x_mag / aspect;
          } else if (x_mag != null) {
            aspect = 1;
            y_mag = x_mag;
          } else if (y_mag != null) {
            aspect = 1;
            x_mag = y_mag;
          } else {
            this._log("Not enough field of view parameters for an orthographic camera.", ColladaLoader2.messageError);
          }
          camera = new THREE.OrthographicCamera(-x_mag, +x_mag, -y_mag, +y_mag, z_min, z_max);
          break;
        case "perspective":
          if ((x_fov != null) && (y_fov != null)) {
            aspect = x_fov / y_fov;
          } else if ((y_fov != null) && (aspect != null)) {
            x_fov = y_fov * aspect;
          } else if ((x_fov != null) && (aspect != null)) {
            y_fov = x_fov / aspect;
          } else if (x_fov != null) {
            aspect = 1;
            y_fov = x_fov;
          } else if (y_fov != null) {
            aspect = 1;
            x_fov = y_fov;
          } else {
            this._log("Not enough field of view parameters for a perspective camera.", ColladaLoader2.messageError);
          }
          camera = new THREE.PerspectiveCamera(y_fov, aspect, z_min, z_max);
          break;
        default:
          this._log("Unknown camera type " + daeInstanceCamera.type + ", camera ignored.", ColladaLoader2.messageError);
      }
      return camera;
    };

    ColladaFile.prototype._createStaticMesh = function(daeInstanceGeometry) {
      var daeGeometry, mesh, threejsGeometry, threejsMaterial, _ref;
      daeGeometry = this._getLinkTarget(daeInstanceGeometry.geometry, ColladaGeometry);
      if (!(daeGeometry != null)) {
        this._log("Geometry instance has no geometry, mesh ignored", ColladaLoader2.messageWarning);
        return null;
      }
      _ref = this._createGeometryAndMaterial(daeGeometry, daeInstanceGeometry.materials), threejsGeometry = _ref[0], threejsMaterial = _ref[1];
      mesh = new THREE.Mesh(threejsGeometry, threejsMaterial);
      return mesh;
    };

    ColladaFile.prototype._createGeometryAndMaterial = function(daeGeometry, daeInstanceMaterials) {
      var material, threejsGeometry, threejsMaterial, threejsMaterials, _i, _len, _ref;
      threejsMaterials = this._createMaterials(daeInstanceMaterials);
      threejsGeometry = this._createGeometry(daeGeometry, threejsMaterials);
      threejsMaterial = null;
      if (threejsMaterials.materials.length > 1) {
        threejsMaterial = new THREE.MeshFaceMaterial();
        _ref = threejsMaterials.materials;
        for (_i = 0, _len = _ref.length; _i < _len; _i++) {
          material = _ref[_i];
          threejsMaterial.materials.push(material);
        }
      } else {
        threejsMaterial = threejsMaterials.materials[0];
      }
      return [threejsGeometry, threejsMaterial];
    };

    ColladaFile.prototype._createAnimatedMesh = function(daeInstanceController, daeController) {
      daeController = this._getLinkTarget(daeInstanceController.controller, ColladaController);
      if (daeController.skin != null) {
        return this._createSkinMesh(daeInstanceController, daeController);
      }
      if (daeController.morph != null) {
        return this._createMorphMesh(daeInstanceController, daeController);
      }
      this._log("Controller has neither a skin nor a morph, can not create a mesh", ColladaLoader2.messageError);
      return null;
    };

    ColladaFile.prototype._createSkinMesh = function(daeInstanceController, daeController) {
      var bone, bones, daeInvBindMatricesSource, daeJointsSource, daeSkin, daeSkinGeometry, i, jointNode, jointSid, mesh, parentBone, skeleton, skeletonLink, skeletonRootNodes, threejsGeometry, threejsMaterial, _i, _j, _k, _l, _len, _len2, _len3, _len4, _ref, _ref2, _ref3, _ref4, _ref5, _ref6, _ref7;
      daeSkin = daeController.skin;
      if (!(daeSkin != null) || !(daeSkin instanceof ColladaSkin)) {
        this._log("Controller for a skinned mesh has no skin, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      daeSkinGeometry = this._getLinkTarget(daeSkin.source);
      if (!(daeSkinGeometry != null)) {
        this._log("Skin for a skinned mesh has no geometry, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      if (!this._options["useAnimations"]) {
        _ref = this._createGeometryAndMaterial(daeSkinGeometry, daeInstanceController.materials), threejsGeometry = _ref[0], threejsMaterial = _ref[1];
        return new THREE.Mesh(threejsGeometry, threejsMaterial);
      }
      skeletonRootNodes = [];
      _ref2 = daeInstanceController.skeletons;
      for (_i = 0, _len = _ref2.length; _i < _len; _i++) {
        skeletonLink = _ref2[_i];
        skeleton = this._getLinkTarget(skeletonLink, ColladaVisualSceneNode);
        if (!(skeleton != null)) {
          this._log("Controller instance for a skinned mesh uses unknown skeleton " + skeleton + ", skeleton ignored", ColladaLoader2.messageError);
          continue;
        }
        skeletonRootNodes.push(skeleton);
      }
      if (skeletonRootNodes.length === 0) {
        this._log("Controller instance for a skinned mesh has no skeleton, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      if (!(daeSkin.joints != null)) {
        this._log("Skin has no joints, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      daeJointsSource = this._getLinkTarget((_ref3 = daeSkin.joints.joints) != null ? _ref3.source : void 0, ColladaSource);
      if (!(daeJointsSource != null) || !(daeJointsSource.data != null)) {
        this._log("Skin has no joints source, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      daeInvBindMatricesSource = this._getLinkTarget((_ref4 = daeSkin.joints.invBindMatrices) != null ? _ref4.source : void 0, ColladaSource);
      if (!(daeInvBindMatricesSource != null) || !(daeInvBindMatricesSource.data != null)) {
        this._log("Skin has no inverse bind matrix source, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      if (daeJointsSource.data.length * 16 !== daeInvBindMatricesSource.data.length) {
        this._log("Skin has an inconsistent length of joint data sources, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      bones = [];
      _ref5 = daeJointsSource.data;
      for (_j = 0, _len2 = _ref5.length; _j < _len2; _j++) {
        jointSid = _ref5[_j];
        jointNode = this._findJointNode(jointSid, skeletonRootNodes);
        if (!(jointNode != null)) {
          this._log("Joint " + jointSid + " not found for skin with skeletons " + ((skeletonRootNodes.map(function(node) {
            return node.id;
          })).join(', ')) + ", mesh ignored", ColladaLoader2.messageError);
          return null;
        }
        bone = this._createBone(jointNode, jointSid, bones);
        _fillMatrix4RowMajor(daeInvBindMatricesSource.data, bone.index * 16, bone.invBindMatrix);
      }
      if (this._options["verboseMessages"]) {
        this._log("Skin contains " + bones.length + " bones", ColladaLoader2.messageInfo);
      }
      i = 0;
      while (i < bones.length) {
        bone = bones[i];
        i = i + 1;
        for (_k = 0, _len3 = bones.length; _k < _len3; _k++) {
          parentBone = bones[_k];
          if (bone.node.parent === parentBone.node) {
            bone.parent = parentBone;
            break;
          }
        }
        if ((bone.node.parent != null) && bone.node.parent instanceof ColladaVisualSceneNode && !(bone.parent != null)) {
          bone.parent = this._createBone(bone.node.parent, "", bones);
        }
      }
      if (this._options["verboseMessages"]) {
        this._log("Skeleton contains " + bones.length + " bones", ColladaLoader2.messageInfo);
      }
      if (!(daeSkin.vertexWeights != null)) {
        this._log("Skin has no vertex weight data, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      if (daeSkin.vertexWeights.joints.source.url !== daeSkin.joints.joints.source.url) {
        this._log("Skin uses different data sources for joints in <joints> and <vertex_weights>, this is not supported by this loader, mesh ignored", ColladaLoader2.messageError);
        return null;
      }
      _ref6 = this._createGeometryAndMaterial(daeSkinGeometry, daeInstanceController.materials), threejsGeometry = _ref6[0], threejsMaterial = _ref6[1];
      if (this._options["convertSkinsToMorphs"]) {
        if (this._addSkinMorphTargets(threejsGeometry, daeSkin, bones, threejsMaterial)) {
          return new THREE.MorphAnimMesh(threejsGeometry, threejsMaterial);
        } else {
          return new THREE.Mesh(threejsGeometry, threejsMaterial);
        }
      } else {
        if (this._addSkinBones(threejsGeometry, daeSkin, bones, threejsMaterial)) {
          mesh = new THREE.SkinnedMesh(threejsGeometry, threejsMaterial);
          mesh.boneInverses = [];
          _ref7 = threejsGeometry.bones;
          for (_l = 0, _len4 = _ref7.length; _l < _len4; _l++) {
            bone = _ref7[_l];
            mesh.boneInverses.push(bone.inverse);
          }
          return mesh;
        } else {
          return new THREE.Mesh(threejsGeometry, threejsMaterial);
        }
      }
      return null;
    };

    ColladaFile.prototype._findJointNode = function(jointSid, skeletonRootNodes) {
      var jointNode, skeleton, _i, _len;
      jointNode = null;
      for (_i = 0, _len = skeletonRootNodes.length; _i < _len; _i++) {
        skeleton = skeletonRootNodes[_i];
        jointNode = this._findSidTarget(skeleton, jointSid);
        if (jointNode != null) break;
      }
      if (jointNode instanceof ColladaVisualSceneNode) {
        return jointNode;
      } else {
        return null;
      }
    };

    ColladaFile.prototype._createBone = function(boneNode, jointSid, bones) {
      var bone, transform, _i, _len, _ref;
      bone = new ThreejsSkeletonBone;
      bone.sid = jointSid;
      bone.node = boneNode;
      _ref = boneNode.transformations;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        transform = _ref[_i];
        if (transform.animTarget.channels.length > 0) {
          bone.isAnimated = true;
          break;
        }
      }
      bone.matrix = new THREE.Matrix4;
      boneNode.getTransformMatrix(bone.matrix);
      bone.index = bones.length;
      bones.push(bone);
      return bone;
    };

    ColladaFile.prototype._addSkinMorphTargets = function(threejsGeometry, daeSkin, bones, threejsMaterial) {
      var bindShapeMatrix, bone, boneIndex, boneWeight, boneWeightIndex, enableWarningInvalidWeight, enableWarningNoBones, i, material, sourceVertex, sourceVertices, srcVertex, tempVertex, timesteps, totalWeight, vertex, vertexCount, vertices, vindex, vwJoints, vwJointsSource, vwV, vwVcount, vwWeights, vwWeightsSource, w, weights, _i, _j, _len, _len2, _len3, _ref, _ref2, _ref3;
      timesteps = this._prepareAnimations(bones);
      if (!timesteps > 0) return null;
      sourceVertices = threejsGeometry.vertices;
      vertexCount = sourceVertices.length;
      vwV = daeSkin.vertexWeights.v;
      vwVcount = daeSkin.vertexWeights.vcount;
      vwJointsSource = this._getLinkTarget(daeSkin.vertexWeights.joints.source);
      vwWeightsSource = this._getLinkTarget(daeSkin.vertexWeights.weights.source);
      vwJoints = vwJointsSource != null ? vwJointsSource.data : void 0;
      vwWeights = vwWeightsSource != null ? vwWeightsSource.data : void 0;
      if (!(vwWeights != null)) {
        this._log("Skin has no weights data, no morph targets added for mesh", ColladaLoader2.messageError);
        return null;
      }
      bindShapeMatrix = new THREE.Matrix4;
      if (daeSkin.bindShapeMatrix != null) {
        bindShapeMatrix = _floatsToMatrix4RowMajor(daeSkin.bindShapeMatrix, 0);
      }
      tempVertex = new THREE.Vector3;
      enableWarningNoBones = true;
      enableWarningInvalidWeight = true;
      for (i = 0, _ref = timesteps - 1; i <= _ref; i += 1) {
        this._updateSkinMatrices(bones, bindShapeMatrix, i);
        vertices = [];
        for (_i = 0, _len = sourceVertices.length; _i < _len; _i++) {
          srcVertex = sourceVertices[_i];
          vertices.push(new THREE.Vector3());
        }
        vindex = 0;
        for (i = 0, _len2 = vertices.length; i < _len2; i++) {
          vertex = vertices[i];
          sourceVertex = sourceVertices[i];
          weights = vwVcount[i];
          totalWeight = 0;
          for (w = 0, _ref2 = weights - 1; w <= _ref2; w += 1) {
            boneIndex = vwV[vindex];
            boneWeightIndex = vwV[vindex + 1];
            vindex += 2;
            boneWeight = vwWeights[boneWeightIndex];
            totalWeight += boneWeight;
            if (boneIndex >= 0) {
              bone = bones[boneIndex];
              tempVertex.copy(sourceVertex);
              tempVertex.applyMatrix4(bone.skinMatrix);
              tempVertex.multiplyScalar(boneWeight);
              vertex.add(tempVertex);
            } else {
              tempVertex.copy(sourceVertex);
              tempVertex.applyMatrix4(bindShapeMatrix);
              tempVertex.multiplyScalar(boneWeight);
              vertex.add(tempVertex);
            }
          }
          if (weights === 0) {
            vertex.copy(sourceVertex);
            if (enableWarningNoBones) {
              this._log("Skinned vertex not influenced by any bone, some vertices will be unskinned", ColladaLoader2.messageWarning);
              enableWarningNoBones = false;
            }
          } else if (!((0.01 < totalWeight && totalWeight < 1e6))) {
            vertex.copy(sourceVertex);
            if (enableWarningInvalidWeight) {
              this._log("Zero or infinite total weight for skinned vertex, some vertices will be unskinned", ColladaLoader2.messageWarning);
              enableWarningInvalidWeight = false;
            }
          } else {
            vertex.multiplyScalar(1 / totalWeight);
          }
        }
        if (vindex !== vwV.length) {
          this._log("Skinning did not consume all weights", ColladaLoader2.messageError);
        }
        threejsGeometry.morphTargets.push({
          name: "target",
          vertices: vertices
        });
      }
      threejsGeometry.computeMorphNormals();
      threejsMaterial.morphTargets = true;
      threejsMaterial.morphNormals = true;
      if (threejsMaterial.materials != null) {
        _ref3 = threejsMaterial.materials;
        for (_j = 0, _len3 = _ref3.length; _j < _len3; _j++) {
          material = _ref3[_j];
          material.morphTargets = true;
          material.morphNormals = true;
        }
      }
      return true;
    };

    ColladaFile.prototype._prepareAnimations = function(bones) {
      var bone, channel, channelTimesteps, hasAnimation, timesteps, transform, _i, _j, _k, _len, _len2, _len3, _ref, _ref2;
      timesteps = null;
      for (_i = 0, _len = bones.length; _i < _len; _i++) {
        bone = bones[_i];
        hasAnimation = false;
        _ref = bone.node.transformations;
        for (_j = 0, _len2 = _ref.length; _j < _len2; _j++) {
          transform = _ref[_j];
          transform.resetAnimation();
          transform.selectAllAnimations();
          _ref2 = transform.animTarget.activeChannels;
          for (_k = 0, _len3 = _ref2.length; _k < _len3; _k++) {
            channel = _ref2[_k];
            hasAnimation = true;
            channelTimesteps = channel.inputData.length;
            if ((timesteps != null) && channelTimesteps !== timesteps) {
              this._log("Inconsistent number of time steps, no morph targets added for mesh. Resample all animations to fix this.", ColladaLoader2.messageError);
              return null;
            }
            timesteps = channelTimesteps;
          }
        }
        if (this._options["verboseMessages"] && !hasAnimation) {
          this._log("Joint '" + bone.sid + "' has no animation channel", ColladaLoader2.messageWarning);
        }
      }
      return timesteps;
    };

    ColladaFile.prototype._updateSkinMatrices = function(bones, bindShapeMatrix, keyframe) {
      var bone, _i, _j, _len, _len2;
      for (_i = 0, _len = bones.length; _i < _len; _i++) {
        bone = bones[_i];
        bone.applyAnimation(keyframe);
      }
      for (_j = 0, _len2 = bones.length; _j < _len2; _j++) {
        bone = bones[_j];
        bone.updateSkinMatrix(bindShapeMatrix);
      }
      return null;
    };

    ColladaFile.prototype._addSkinBones = function(threejsGeometry, daeSkin, bones, threejsMaterial) {
      var bindShapeMatrix, bone, boneIndex, boneWeight, boneWeightIndex, bonesPerVertex, enableWarningInvalidWeight, enableWarningTooManyBones, i, indices, key, keyframe, material, pos, rot, scl, sourceVertices, threejsAnimation, threejsBone, threejsBoneAnimation, threejsBones, threejsSkinIndices, threejsSkinWeights, timesteps, totalWeight, vertex, vertexCount, vindex, vwJoints, vwJointsSource, vwV, vwVcount, vwWeights, vwWeightsSource, w, weightCount, weights, _i, _j, _k, _len, _len2, _len3, _len4, _ref, _ref2, _ref3, _ref4, _ref5;
      timesteps = this._prepareAnimations(bones);
      if (!timesteps > 0) return null;
      sourceVertices = threejsGeometry.vertices;
      vertexCount = sourceVertices.length;
      vwV = daeSkin.vertexWeights.v;
      vwVcount = daeSkin.vertexWeights.vcount;
      vwJointsSource = this._getLinkTarget(daeSkin.vertexWeights.joints.source);
      vwWeightsSource = this._getLinkTarget(daeSkin.vertexWeights.weights.source);
      vwJoints = vwJointsSource != null ? vwJointsSource.data : void 0;
      vwWeights = vwWeightsSource != null ? vwWeightsSource.data : void 0;
      if (!(vwWeights != null)) {
        this._log("Skin has no weights data, no skin added for mesh", ColladaLoader2.messageError);
        return null;
      }
      bindShapeMatrix = new THREE.Matrix4;
      if (daeSkin.bindShapeMatrix != null) {
        bindShapeMatrix = _floatsToMatrix4RowMajor(daeSkin.bindShapeMatrix, 0);
      }
      pos = new THREE.Vector3();
      rot = new THREE.Quaternion();
      scl = new THREE.Vector3();
      enableWarningTooManyBones = true;
      enableWarningInvalidWeight = true;
      threejsSkinIndices = [];
      threejsSkinWeights = [];
      vindex = 0;
      bonesPerVertex = 4;
      indices = [0, 0, 0, 0];
      weights = [0, 0, 0, 0];
      for (i = 0, _len = sourceVertices.length; i < _len; i++) {
        vertex = sourceVertices[i];
        weightCount = vwVcount[i];
        if (weightCount > bonesPerVertex) {
          if (enableWarningTooManyBones) {
            this._log("Too many bones influence a vertex, some influences will be discarded. Threejs supports only " + bonesPerVertex + " bones per vertex.", ColladaLoader2.messageWarning);
            enableWarningTooManyBones = false;
          }
          weightCount = bonesPerVertex;
        }
        totalWeight = 0;
        for (w = 0, _ref = weightCount - 1; w <= _ref; w += 1) {
          boneIndex = vwV[vindex];
          boneWeightIndex = vwV[vindex + 1];
          vindex += 2;
          boneWeight = vwWeights[boneWeightIndex];
          totalWeight += boneWeight;
          indices[w] = boneIndex;
          weights[w] = boneWeight;
        }
        for (w = weights, _ref2 = bonesPerVertex - 1; w <= _ref2; w += 1) {
          indices[w] = 0;
          weights[w] = 0;
        }
        if (!((0.01 < totalWeight && totalWeight < 1e6))) {
          if (enableWarningInvalidWeight) {
            this._log("Zero or infinite total weight for skinned vertex, skin will be broken", ColladaLoader2.messageWarning);
            enableWarningInvalidWeight = false;
          }
        } else {
          for (w = 0, _ref3 = bonesPerVertex - 1; w <= _ref3; w += 1) {
            weights[w] /= totalWeight;
          }
        }
        threejsSkinIndices.push(new THREE.Vector4(indices[0], indices[1], indices[2], indices[3]));
        threejsSkinWeights.push(new THREE.Vector4(weights[0], weights[1], weights[2], weights[3]));
      }
      threejsGeometry.skinIndices = threejsSkinIndices;
      threejsGeometry.skinWeights = threejsSkinWeights;
      threejsBones = [];
      for (_i = 0, _len2 = bones.length; _i < _len2; _i++) {
        bone = bones[_i];
        threejsBone = {};
        if (bone.parent != null) {
          threejsBone.parent = bone.parent.index;
        } else {
          threejsBone.parent = -1;
        }
        threejsBone.name = bone.node.name;
        bone.matrix.decompose(pos, rot, scl);
        threejsBone.pos = [pos.x, pos.y, pos.z];
        threejsBone.scl = [scl.x, scl.y, scl.z];
        threejsBone.rotq = [rot.x, rot.y, rot.z, rot.w];
        threejsBone.rot = null;
        threejsBone.inverse = new THREE.Matrix4;
        threejsBone.inverse.multiplyMatrices(bone.invBindMatrix, bindShapeMatrix);
        threejsBones.push(threejsBone);
      }
      threejsGeometry.bones = threejsBones;
      threejsAnimation = {};
      threejsAnimation.name = "animation";
      threejsAnimation.hierarchy = [];
      for (_j = 0, _len3 = bones.length; _j < _len3; _j++) {
        bone = bones[_j];
        threejsBoneAnimation = {};
        threejsBoneAnimation.parent = bone.index;
        threejsBoneAnimation.keys = [];
        for (keyframe = 0, _ref4 = timesteps - 1; keyframe <= _ref4; keyframe += 1) {
          bone.applyAnimation(keyframe);
          bone.updateSkinMatrix(bindShapeMatrix);
          key = {};
          key.time = keyframe;
          bone.matrix.decompose(pos, rot, scl);
          key.pos = [pos.x, pos.y, pos.z];
          key.scl = [scl.x, scl.y, scl.z];
          key.rot = [rot.x, rot.y, rot.z, rot.w];
          threejsBoneAnimation.keys.push(key);
        }
        threejsAnimation.hierarchy.push(threejsBoneAnimation);
      }
      threejsAnimation.fps = 30;
      threejsAnimation.length = timesteps - 1;
      threejsGeometry.animation = threejsAnimation;
      threejsMaterial.skinning = true;
      if (threejsMaterial.materials != null) {
        _ref5 = threejsMaterial.materials;
        for (_k = 0, _len4 = _ref5.length; _k < _len4; _k++) {
          material = _ref5[_k];
          material.skinning = true;
        }
      }
      return true;
    };

    ColladaFile.prototype._createMorphMesh = function(daeInstanceController, daeController) {
      this._log("Morph animated meshes not supported, mesh ignored", ColladaLoader2.messageError);
      return null;
    };

    ColladaFile.prototype._createGeometry = function(daeGeometry, materials) {
      var materialIndex, threejsGeometry, triangles, _i, _len, _ref;
      threejsGeometry = new THREE.Geometry();
      _ref = daeGeometry.triangles;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        triangles = _ref[_i];
        materialIndex = materials.indices[triangles.material];
        if (!(materialIndex != null)) {
          this._log("Material symbol " + triangles.material + " has no bound material instance", ColladaLoader2.messageError);
          materialIndex = 0;
        }
        this._addTrianglesToGeometry(daeGeometry, triangles, materialIndex, threejsGeometry);
      }
      threejsGeometry.computeFaceNormals();
      threejsGeometry.computeCentroids();
      if (materials.needtangents) threejsGeometry.computeTangents();
      threejsGeometry.computeBoundingBox();
      return threejsGeometry;
    };

    ColladaFile.prototype._addTrianglesToGeometry = function(daeGeometry, triangles, materialIndex, threejsGeometry) {
      var baseOffset0, baseOffset1, baseOffset2, c, color, data, dataTriColor, dataTriNormal, dataTriTexcoord, dataVertColor, dataVertNormal, dataVertPos, dataVertTexcoord, face, faceVertexUvs, i, indices, input, inputTriColor, inputTriNormal, inputTriTexcoord, inputTriVertices, inputVertColor, inputVertNormal, inputVertPos, inputVertTexcoord, missingFaces, n0, n1, n2, normal, numExistingFaces, numExistingTexcoordSets, numNewFaces, numNewTexcoordSets, srcTriColor, srcTriNormal, srcTriTexcoord, srcTriVertices, srcVertColor, srcVertNormal, srcVertPos, srcVertTexcoord, t0, t1, t2, texcoord, triangleBaseOffset, triangleStride, v0, v1, v2, vcount, vertexStride, _i, _j, _k, _len, _len2, _len3, _len4, _len5, _len6, _len7, _ref, _ref2, _ref3, _ref4, _step, _unused,
        _this = this;
      inputTriVertices = null;
      inputTriNormal = null;
      inputTriColor = null;
      inputTriTexcoord = [];
      _ref = triangles.inputs;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        input = _ref[_i];
        switch (input.semantic) {
          case "VERTEX":
            inputTriVertices = input;
            break;
          case "NORMAL":
            inputTriNormal = input;
            break;
          case "COLOR":
            inputTriColor = input;
            break;
          case "TEXCOORD":
            inputTriTexcoord.push(input);
            break;
          default:
            this._log("Unknown triangles input semantic " + input.semantic + " ignored", ColladaLoader2.messageWarning);
        }
      }
      srcTriVertices = this._getLinkTarget(inputTriVertices.source, ColladaVertices);
      if (!(srcTriVertices != null)) {
        this._log("Geometry " + daeGeometry.id + " has no vertices", ColladaLoader2.messageError);
        return;
      }
      srcTriNormal = this._getLinkTarget(inputTriNormal != null ? inputTriNormal.source : void 0, ColladaSource);
      srcTriColor = this._getLinkTarget(inputTriColor != null ? inputTriColor.source : void 0, ColladaSource);
      srcTriTexcoord = inputTriTexcoord.map(function(x) {
        return _this._getLinkTarget(x != null ? x.source : void 0, ColladaSource);
      });
      inputVertPos = null;
      inputVertNormal = null;
      inputVertColor = null;
      inputVertTexcoord = [];
      _ref2 = srcTriVertices.inputs;
      for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
        input = _ref2[_j];
        switch (input.semantic) {
          case "POSITION":
            inputVertPos = input;
            break;
          case "NORMAL":
            inputVertNormal = input;
            break;
          case "COLOR":
            inputVertColor = input;
            break;
          case "TEXCOORD":
            inputVertTexcoord.push(input);
            break;
          default:
            this._log("Unknown vertices input semantic " + input.semantic + " ignored", ColladaLoader2.messageWarning);
        }
      }
      srcVertPos = this._getLinkTarget(inputVertPos.source, ColladaSource);
      if (!(srcVertPos != null)) {
        this._log("Geometry " + daeGeometry.id + " has no vertex positions", ColladaLoader2.messageError);
        return;
      }
      srcVertNormal = this._getLinkTarget(inputVertNormal != null ? inputVertNormal.source : void 0, ColladaSource);
      srcVertColor = this._getLinkTarget(inputVertColor != null ? inputVertColor.source : void 0, ColladaSource);
      srcVertTexcoord = inputVertTexcoord.map(function(x) {
        return _this._getLinkTarget(x != null ? x.source : void 0, ColladaSource);
      });
      dataVertPos = this._createVector3Array(srcVertPos);
      dataVertNormal = this._createVector3Array(srcVertNormal);
      dataTriNormal = this._createVector3Array(srcTriNormal);
      dataVertColor = this._createColorArray(srcVertColor);
      dataTriColor = this._createColorArray(srcTriColor);
      dataVertTexcoord = srcVertTexcoord.map(function(x) {
        return _this._createUVArray(x);
      });
      dataTriTexcoord = srcTriTexcoord.map(function(x) {
        return _this._createUVArray(x);
      });
      threejsGeometry.vertices = dataVertPos;
      numNewTexcoordSets = dataVertTexcoord.length + dataTriTexcoord.length;
      numExistingTexcoordSets = threejsGeometry.faceVertexUvs.length;
      numNewFaces = triangles.count;
      numExistingFaces = threejsGeometry.faces.count;
      _ref3 = threejsGeometry.faceVertexUvs;
      for (i = 0, _len3 = _ref3.length; i < _len3; i++) {
        faceVertexUvs = _ref3[i];
        if (i < numNewTexcoordSets) {
          missingFaces = faceVertexUvs.length - threejsGeometry.faces.length;
          this._addEmptyUVs(faceVertexUvs, missingFaces);
        } else {
          missingFaces = faceVertexUvs.length - threejsGeometry.faces.length + numNewFaces;
          this._addEmptyUVs(faceVertexUvs, missingFaces);
        }
      }
      while (threejsGeometry.faceVertexUvs.length < numNewTexcoordSets) {
        faceVertexUvs = [];
        this._addEmptyUVs(faceVertexUvs, numExistingFaces);
        threejsGeometry.faceVertexUvs.push(faceVertexUvs);
      }
      if (triangles.type !== "triangles") {
        vcount = triangles.vcount;
        for (_k = 0, _len4 = vcount.length; _k < _len4; _k++) {
          c = vcount[_k];
          if (c !== 3) {
            this._log("Geometry " + daeGeometry.id + " has non-triangle polygons, geometry ignored", ColladaLoader2.messageError);
            return;
          }
        }
      }
      indices = triangles.indices;
      triangleStride = indices.length / triangles.count;
      vertexStride = triangleStride / 3;
      _ref4 = triangles.indices;
      for (triangleBaseOffset = 0, _len5 = _ref4.length, _step = triangleStride; triangleBaseOffset < _len5; triangleBaseOffset += _step) {
        _unused = _ref4[triangleBaseOffset];
        baseOffset0 = triangleBaseOffset + 0 * vertexStride;
        baseOffset1 = triangleBaseOffset + 1 * vertexStride;
        baseOffset2 = triangleBaseOffset + 2 * vertexStride;
        v0 = indices[baseOffset0 + inputTriVertices.offset];
        v1 = indices[baseOffset1 + inputTriVertices.offset];
        v2 = indices[baseOffset2 + inputTriVertices.offset];
        if (dataVertNormal != null) {
          normal = [dataVertNormal[v0], dataVertNormal[v1], dataVertNormal[v2]];
        } else if (dataTriNormal != null) {
          n0 = indices[baseOffset0 + inputTriNormal.offset];
          n1 = indices[baseOffset1 + inputTriNormal.offset];
          n2 = indices[baseOffset2 + inputTriNormal.offset];
          normal = [dataTriNormal[n0], dataTriNormal[n1], dataTriNormal[n2]];
        } else {
          normal = null;
        }
        if (dataVertColor != null) {
          color = [dataVertColor[v0], dataVertColor[v1], dataVertColor[v2]];
        } else if (dataTriColor != null) {
          n0 = indices[baseOffset0 + inputTriColor.offset];
          n1 = indices[baseOffset1 + inputTriColor.offset];
          n2 = indices[baseOffset2 + inputTriColor.offset];
          color = [dataTriColor[n0], dataTriColor[n1], dataTriColor[n2]];
        } else {
          color = null;
        }
        face = new THREE.Face3(v0, v1, v2, normal, color);
        if (materialIndex != null) face.materialIndex = materialIndex;
        threejsGeometry.faces.push(face);
        for (i = 0, _len6 = dataVertTexcoord.length; i < _len6; i++) {
          data = dataVertTexcoord[i];
          if (!(data != null)) {
            geometry.faceVertexUvs[i].push([new THREE.Vector2(0, 0), new THREE.Vector2(0, 0), new THREE.Vector2(0, 0)]);
          } else {
            texcoord = [data[v0], data[v1], data[v2]];
            geometry.faceVertexUvs[i].push(texcoord);
          }
        }
        for (i = 0, _len7 = dataTriTexcoord.length; i < _len7; i++) {
          data = dataTriTexcoord[i];
          if (!(data != null)) {
            geometry.faceVertexUvs[i].push([new THREE.Vector2(0, 0), new THREE.Vector2(0, 0), new THREE.Vector2(0, 0)]);
          } else {
            t0 = indices[baseOffset0 + inputTriTexcoord[i].offset];
            t1 = indices[baseOffset1 + inputTriTexcoord[i].offset];
            t2 = indices[baseOffset2 + inputTriTexcoord[i].offset];
            texcoord = [data[t0], data[t1], data[t2]];
            threejsGeometry.faceVertexUvs[i].push(texcoord);
          }
        }
      }
    };

    ColladaFile.prototype._addEmptyUVs = function(faceVertexUvs, count) {
      var i, _ref;
      for (i = 0, _ref = count - 1; i <= _ref; i += 1) {
        faceVertexUvs.push(new THREE.Vector2(0, 0));
      }
    };

    ColladaFile.prototype._createVector3Array = function(source) {
      var data, i, srcData, _ref;
      if (!(source != null)) return null;
      if (source.stride !== 3) {
        this._log("Vector source data does not contain 3D vectors", ColladaLoader2.messageError);
        return null;
      }
      data = [];
      srcData = source.data;
      for (i = 0, _ref = srcData.length - 1; i <= _ref; i += 3) {
        data.push(new THREE.Vector3(srcData[i], srcData[i + 1], srcData[i + 2]));
      }
      return data;
    };

    ColladaFile.prototype._createColorArray = function(source) {
      var data, i, srcData, _ref, _ref2;
      if (!(source != null)) return null;
      if (source.stride < 3) {
        this._log("Color source data does not contain 3+D vectors", ColladaLoader2.messageError);
        return null;
      }
      data = [];
      srcData = source.data;
      for (i = 0, _ref = srcData.length - 1, _ref2 = source.stride; 0 <= _ref ? i <= _ref : i >= _ref; i += _ref2) {
        data.push(new THREE.Color().setRGB(srcData[i], srcData[i + 1], srcData[i + 2]));
      }
      return data;
    };

    ColladaFile.prototype._createUVArray = function(source) {
      var data, i, srcData, _ref, _ref2;
      if (!(source != null)) return null;
      if (source.stride < 2) {
        this._log("UV source data does not contain 2+D vectors", ColladaLoader2.messageError);
        return null;
      }
      data = [];
      srcData = source.data;
      for (i = 0, _ref = srcData.length - 1, _ref2 = source.stride; 0 <= _ref ? i <= _ref : i >= _ref; i += _ref2) {
        data.push(new THREE.Vector2(srcData[i], 1.0 - srcData[i + 1]));
      }
      return data;
    };

    ColladaFile.prototype._createMaterials = function(daeInstanceMaterials) {
      var daeInstanceMaterial, numMaterials, result, symbol, threejsMaterial, _i, _len;
      result = new ThreejsMaterialMap;
      numMaterials = 0;
      for (_i = 0, _len = daeInstanceMaterials.length; _i < _len; _i++) {
        daeInstanceMaterial = daeInstanceMaterials[_i];
        symbol = daeInstanceMaterial.symbol;
        if (!(symbol != null)) {
          this._log("Material instance has no symbol, material skipped.", ColladaLoader2.messageError);
          continue;
        }
        if (result.indices[symbol] != null) {
          this._log("Geometry instance tried to map material symbol " + symbol + " multiple times", ColladaLoader2.messageError);
          continue;
        }
        threejsMaterial = this._createMaterial(daeInstanceMaterial);
        if ((threejsMaterial.bumpMap != null) || (threejsMaterial.normalMap != null)) {
          result.needtangents = true;
        }
        this.threejs.materials.push(threejsMaterial);
        result.materials.push(threejsMaterial);
        result.indices[symbol] = numMaterials++;
      }
      return result;
    };

    ColladaFile.prototype._createMaterial = function(daeInstanceMaterial) {
      var daeEffect, daeMaterial;
      daeMaterial = this._getLinkTarget(daeInstanceMaterial.material, ColladaMaterial);
      if (!(daeMaterial != null)) return this._createDefaultMaterial;
      daeEffect = this._getLinkTarget(daeMaterial.effect, ColladaEffect);
      if (!(daeEffect != null)) return this._createDefaultMaterial;
      return this._createBuiltInMaterial(daeEffect);
    };

    ColladaFile.prototype._createShaderMaterial = function(daeEffect) {
      var materialNormalMap, shader, technique, textureDiffuse, textureLight, textureNormal, textureSpecular, uniforms, _ref, _ref2, _ref3;
      technique = daeEffect.technique;
      shader = THREE.ShaderUtils.lib["normal"];
      uniforms = THREE.UniformsUtils.clone(shader.uniforms);
      textureNormal = this._loadThreejsTexture(technique.bump);
      if (textureNormal != null) {
        uniforms["tNormal"].texture = textureNormal;
        uniforms["uNormalScale"].value = 0.85;
      }
      textureDiffuse = this._loadThreejsTexture(technique.diffuse);
      if (textureDiffuse != null) {
        uniforms["tDiffuse"].texture = textureDiffuse;
        uniforms["enableDiffuse"].value = true;
      } else {
        uniforms["enableDiffuse"].value = false;
      }
      textureSpecular = this._loadThreejsTexture(technique.specular);
      if (textureSpecular != null) {
        uniforms["tSpecular"].texture = textureSpecular;
        uniforms["enableSpecular"].value = true;
      } else {
        uniforms["enableSpecular"].value = false;
      }
      textureLight = this._loadThreejsTexture(technique.emission);
      if (textureLight != null) {
        uniforms["tAO"].texture = textureLight;
        uniforms["enableAO"].value = true;
      } else {
        uniforms["enableAO"].value = false;
      }
      if (((_ref = technique.diffuse) != null ? _ref.color : void 0) != null) {
        uniforms["uDiffuseColor"].value.setHex(_colorToHex(technique.diffuse.color));
      }
      if (((_ref2 = technique.specular) != null ? _ref2.color : void 0) != null) {
        uniforms["uSpecularColor"].value.setHex(_colorToHex(technique.specular.color));
      }
      if (((_ref3 = technique.ambient) != null ? _ref3.color : void 0) != null) {
        uniforms["uAmbientColor"].value.setHex(_colorToHex(technique.ambient.color));
      }
      if (technique.shininess != null) {
        uniforms["uShininess"].value = technique.shininess;
      }
      if (technique.transparency != null) {
        uniforms["uOpacity"].value = this._getOpacity(daeEffect);
      }
      materialNormalMap = new THREE.ShaderMaterial({
        fragmentShader: shader.fragmentShader,
        vertexShader: shader.vertexShader,
        uniforms: uniforms,
        lights: true
      });
      return materialNormalMap;
    };

    ColladaFile.prototype._getOpacity = function(daeEffect) {
      var opacityMode, technique, transparency, transparent, transparentA, _ref;
      technique = daeEffect.technique;
      transparent = technique.transparent;
      opacityMode = transparent != null ? transparent.opaque : void 0;
      if ((opacityMode != null) && opacityMode !== "A_ONE") {
        this._log("Opacity mode " + opacityMode + " not supported, transparency will be broken", ColladaLoader2.messageWarning);
      }
      if ((transparent != null ? transparent.textureSampler : void 0) != null) {
        this._log("Separate transparency texture not supported, transparency will be broken", ColladaLoader2.messageWarning);
      }
      transparentA = (transparent != null ? (_ref = transparent.color) != null ? _ref[3] : void 0 : void 0) || 1;
      transparency = technique.transparency || 1;
      return transparentA * transparency;
    };

    ColladaFile.prototype._hasTransparency = function(daeEffect) {
      var technique, _ref, _ref2;
      technique = daeEffect.technique;
      return (((_ref = technique.transparent) != null ? _ref.textureSampler : void 0) != null) || ((0 >= (_ref2 = technique.transparency) && _ref2 >= 1));
    };

    ColladaFile.prototype._createBuiltInMaterial = function(daeEffect) {
      var hasTransparency, opacity, params, technique;
      technique = daeEffect.technique;
      params = {};
      this._setThreejsMaterialParam(params, technique.diffuse, "diffuse", "map", false);
      this._setThreejsMaterialParam(params, technique.emission, "emissive", null, false);
      this._setThreejsMaterialParam(params, technique.ambient, "ambient", "lightMap", false);
      this._setThreejsMaterialParam(params, technique.specular, "specular", "specularMap", false);
      this._setThreejsMaterialParam(params, technique.bump, null, "normalMap", false);
      if (params["bumpMap"]) params["bumpScale"] = 1.0;
      if (params["normalMap"]) params["normalScale"] = new THREE.Vector2(1.0, 1.0);
      if (params["map"] != null) params["diffuse"] = 0xffffff;
      if (params["specularMap"] != null) params["specular"] = 0xffffff;
      if (!(params["diffuse"] != null)) params["diffuse"] = 0xffffff;
      if (technique.shininess != null) params["shininess"] = technique.shininess;
      if (technique.reflectivity != null) {
        params["reflectivity"] = technique.reflectivity;
      }
      hasTransparency = this._hasTransparency(daeEffect);
      if (hasTransparency) {
        params["transparent"] = true;
        opacity = this._getOpacity(daeEffect);
        params["opacity"] = opacity;
        params["alphaTest"] = 0.001;
      }
      if (technique.doubleSided) params["side"] = THREE.DoubleSide;
      params["shading"] = THREE.SmoothShading;
      params["perPixel"] = true;
      switch (technique.shading) {
        case "blinn":
        case "phong":
          params["color"] = params["diffuse"];
          return new THREE.MeshPhongMaterial(params);
        case "lambert":
          params["color"] = params["diffuse"];
          return new THREE.MeshLambertMaterial(params);
        case "constant":
          params["color"] = params["emission"];
          return new THREE.MeshBasicMaterial(params);
        default:
          return this._createDefaultMaterial;
      }
    };

    ColladaFile.prototype._createDefaultMaterial = function() {
      return new THREE.MeshLambertMaterial({
        color: 0xdddddd,
        shading: THREE.FlatShading
      });
    };

    ColladaFile.prototype._setThreejsMaterialParam = function(params, colorOrTexture, nameColor, nameTexture, replace) {
      var threejsTexture;
      if (!(colorOrTexture != null)) return;
      if ((colorOrTexture.color != null) && (nameColor != null)) {
        if (!replace && (params[nameColor] != null)) return;
        params[nameColor] = _colorToHex(colorOrTexture.color);
      } else if ((colorOrTexture.textureSampler != null) && (nameTexture != null)) {
        if (!replace && (params[nameTexture] != null)) return;
        threejsTexture = this._loadThreejsTexture(colorOrTexture);
        if (threejsTexture != null) params[nameTexture] = threejsTexture;
      }
    };

    ColladaFile.prototype._loadThreejsTexture = function(colorOrTexture) {
      var imageURL, texture, textureImage, textureSampler, textureSurface;
      if (!(colorOrTexture.textureSampler != null)) return null;
      textureSampler = this._getLinkTarget(colorOrTexture.textureSampler, ColladaEffectSampler);
      if (!(textureSampler != null)) return null;
      textureImage = null;
      if (textureSampler.image != null) {
        textureImage = this._getLinkTarget(textureSampler.image, ColladaImage);
      } else if (textureSampler.surface != null) {
        textureSurface = this._getLinkTarget(textureSampler.surface, ColladaEffectSurface);
        textureImage = this._getLinkTarget(textureSurface.initFrom, ColladaImage);
      }
      if (!(textureImage != null)) return null;
      imageURL = this._baseUrl + textureImage.initFrom;
      texture = this._loader._loadTextureFromURL(imageURL);
      return texture;
    };

    return ColladaFile;

  })();

  ColladaLoader2 = (function() {

    ColladaLoader2.messageTrace = 0;

    ColladaLoader2.messageInfo = 1;

    ColladaLoader2.messageWarning = 2;

    ColladaLoader2.messageError = 3;

    ColladaLoader2.messageTypes = ["TRACE", "INFO", "WARNING", "ERROR"];

    function ColladaLoader2() {
      this.log = ColladaLoader2.logConsole;
      this._imageCache = {};
      this.options = {
        "useAnimations": true,
        "convertSkinsToMorphs": false,
        "verboseMessages": false,
        "localImageMode": false
      };
    }

    ColladaLoader2.logConsole = function(msg, type) {
      console.log("ColladaLoader2 " + ColladaLoader2.messageTypes[type] + ": " + msg);
    };

    ColladaLoader2.prototype.setLog = function(logCallback) {
      this.log = logCallback || this.logConsole;
    };

    ColladaLoader2.prototype.addChachedTextures = function(textures) {
      var key, value;
      for (key in textures) {
        value = textures[key];
        this._imageCache[key] = value;
      }
    };

    ColladaLoader2.prototype.load = function(url, readyCallback, progressCallback) {
      var length, req, _ref,
        _this = this;
      length = 0;
      if ((_ref = document.implementation) != null ? _ref.createDocument : void 0) {
        req = new XMLHttpRequest();
        if (typeof req.overrideMimeType === "function") {
          req.overrideMimeType("text/xml");
        }
        req.onreadystatechange = function() {
          if (req.readyState === 4) {
            if (req.status === 0 || req.status === 200) {
              if (req.responseXML) {
                return _this.parse(req.responseXML, readyCallback, url);
              } else {
                return _this.log("Empty or non-existing file " + url + ".", ColladaLoader2.messageError);
              }
            }
          } else if (req.readyState === 3) {
            if (progressCallback) {
              if (length === 0) length = req.getResponseHeader("Content-Length");
              return progressCallback({
                total: length,
                loaded: req.responseText.length
              });
            }
          }
        };
        req.open("GET", url, true);
        req.send(null);
      } else {
        this.log("Don't know how to parse XML!", ColladaLoader2.messageError);
      }
    };

    ColladaLoader2.prototype.parse = function(doc, readyCallback, url) {
      var file;
      file = new ColladaFile(this);
      file.setUrl(url);
      file._readyCallback = readyCallback;
      file._parseXml(doc);
      file._linkAnimations();
      file._createSceneGraph();
      if (file._readyCallback) file._readyCallback(file);
      return file;
    };

    ColladaLoader2.prototype._loadTextureFromURL = function(imageURL) {
      var texture;
      texture = this._imageCache[imageURL];
      if (texture != null) return texture;
      if (this.options.localImageMode) texture = this._loadImageLocal(imageURL);
      if (!(texture != null)) texture = this._loadImageSimple(imageURL);
      if (texture != null) {
        this._imageCache[imageURL] = texture;
      } else {
        this.log("Texture " + imageURL + " could not be loaded, texture will be ignored.", ColladaLoader2.messageError);
      }
      return texture;
    };

    ColladaLoader2.prototype._loadImageThreejs = function(imageURL) {
      var texture;
      texture = THREE.ImageUtils.loadTexture(imageURL);
      texture.flipY = false;
      return texture;
    };

    ColladaLoader2.prototype._loadImageSimple = function(imageURL) {
      var image, texture;
      image = new Image();
      texture = new THREE.Texture(image);
      texture.flipY = false;
      texture.wrapS = THREE.RepeatWrapping;
      texture.wrapT = THREE.RepeatWrapping;
      image.onload = function() {
        return texture.needsUpdate = true;
      };
      image.crossOrigin = 'anonymous';
      image.src = imageURL;
      return texture;
    };

    ColladaLoader2.prototype._loadImageLocal = function(imageURL) {
      var cachedURLBase, imageURLBase, key, texture, value, _ref, _ref2;
      imageURLBase = this._removeSameDirectoryPath(imageURL);
      _ref = this._imageCache;
      for (key in _ref) {
        value = _ref[key];
        cachedURLBase = this._removeSameDirectoryPath(key);
        if (imageURLBase.indexOf(cachedURLBase) >= 0) {
          texture = value;
          break;
        }
      }
      imageURLBase = this._removeSameDirectoryPath(this._removeFileExtension(imageURL));
      if (!(texture != null)) {
        _ref2 = this._imageCache;
        for (key in _ref2) {
          value = _ref2[key];
          cachedURLBase = this._removeSameDirectoryPath(this._removeFileExtension(key));
          if (imageURLBase.indexOf(cachedURLBase) >= 0) {
            texture = value;
            break;
          }
        }
      }
      return texture;
    };

    ColladaLoader2.prototype._removeFileExtension = function(filePath) {
      return filePath.substr(0, filePath.lastIndexOf(".")) || filePath;
    };

    ColladaLoader2.prototype._removeSameDirectoryPath = function(filePath) {
      return filePath.replace(/^.\//, "");
    };

    return ColladaLoader2;

  })();

  _strToStrings = function(str) {
    var trimmed;
    if (str.length > 0) {
      trimmed = str.trim();
      return trimmed.split(/\s+/);
    } else {
      return [];
    }
  };

  _strToFloats = function(str) {
    var data, i, string, strings, _len;
    strings = _strToStrings(str);
    data = new Float32Array(strings.length);
    for (i = 0, _len = strings.length; i < _len; i++) {
      string = strings[i];
      data[i] = parseFloat(string);
    }
    return data;
  };

  _strToInts = function(str) {
    var data, i, string, strings, _len;
    strings = _strToStrings(str);
    data = new Int32Array(strings.length);
    for (i = 0, _len = strings.length; i < _len; i++) {
      string = strings[i];
      data[i] = parseInt(string, 10);
    }
    return data;
  };

  _strToBools = function(str) {
    var data, i, string, strings, _len, _ref;
    strings = _strToStrings(str);
    data = new Uint8Array(strings.length);
    for (i = 0, _len = strings.length; i < _len; i++) {
      string = strings[i];
      data[i] = (_ref = string === "true" || string === "1") != null ? _ref : {
        1: 0
      };
    }
    return data;
  };

  _strToColor = function(str) {
    var rgba;
    rgba = _strToFloats(str);
    if (rgba.length === 4) {
      return rgba;
    } else {
      return null;
    }
  };

  _colorToHex = function(rgba) {
    if (rgba != null) {
      return Math.floor(rgba[0] * 255) << 16 ^ Math.floor(rgba[1] * 255) << 8 ^ Math.floor(rgba[2] * 255);
    } else {
      return null;
    }
  };

  _floatsToMatrix4ColumnMajor = function(data, offset) {
    return new THREE.Matrix4(data[0 + offset], data[4 + offset], data[8 + offset], data[12 + offset], data[1 + offset], data[5 + offset], data[9 + offset], data[13 + offset], data[2 + offset], data[6 + offset], data[10 + offset], data[14 + offset], data[3 + offset], data[7 + offset], data[11 + offset], data[15 + offset]);
  };

  _floatsToMatrix4RowMajor = function(data, offset) {
    return new THREE.Matrix4(data[0 + offset], data[1 + offset], data[2 + offset], data[3 + offset], data[4 + offset], data[5 + offset], data[6 + offset], data[7 + offset], data[8 + offset], data[9 + offset], data[10 + offset], data[11 + offset], data[12 + offset], data[13 + offset], data[14 + offset], data[15 + offset]);
  };

  _fillMatrix4ColumnMajor = function(data, offset, matrix) {
    return matrix.set(data[0 + offset], data[4 + offset], data[8 + offset], data[12 + offset], data[1 + offset], data[5 + offset], data[9 + offset], data[13 + offset], data[2 + offset], data[6 + offset], data[10 + offset], data[14 + offset], data[3 + offset], data[7 + offset], data[11 + offset], data[15 + offset]);
  };

  _fillMatrix4RowMajor = function(data, offset, matrix) {
    return matrix.set(data[0 + offset], data[1 + offset], data[2 + offset], data[3 + offset], data[4 + offset], data[5 + offset], data[6 + offset], data[7 + offset], data[8 + offset], data[9 + offset], data[10 + offset], data[11 + offset], data[12 + offset], data[13 + offset], data[14 + offset], data[15 + offset]);
  };

  _checkMatrix4 = function(matrix) {
    var col1len, col2len, col3len, me;
    me = matrix.elements;
    if (me[3] !== 0 || me[7] !== 0 || me[11] !== 0 || me[15] !== 1) {
      throw new Error("Last row isnt [0,0,0,1]");
    }
    col1len = Math.sqrt(me[0] * me[0] + me[1] * me[1] + me[2] * me[2]);
    col2len = Math.sqrt(me[4] * me[4] + me[5] * me[5] + me[6] * me[6]);
    col3len = Math.sqrt(me[8] * me[8] + me[9] * me[9] + me[10] * me[10]);
    if (col1len < 0.9 || col1len > 1.1) {
      throw new Error("First column has significant scaling");
    }
    if (col2len < 0.9 || col2len > 1.1) {
      throw new Error("Second column has significant scaling");
    }
    if (col3len < 0.9 || col3len > 1.1) {
      throw new Error("Third column has significant scaling");
    }
  };

  _floatsToVec3 = function(data) {
    return new THREE.Vector3(data[0], data[1], data[2]);
  };

  TO_RADIANS = Math.PI / 180.0;

  ColladaLoader2.prototype['setLog'] = ColladaLoader2.prototype.setLog;

  ColladaLoader2.prototype['addChachedTextures'] = ColladaLoader2.prototype.addChachedTextures;

  ColladaLoader2.prototype['load'] = ColladaLoader2.prototype.load;

  ColladaLoader2.prototype['parse'] = ColladaLoader2.prototype.parse;

  if (typeof module !== "undefined" && module !== null) {
    module['exports'] = ColladaLoader2;
  } else if (typeof window !== "undefined" && window !== null) {
    window['ColladaLoader2'] = ColladaLoader2;
  }

}).call(this);
