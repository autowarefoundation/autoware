/*
* Event
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * A collection of Classes that are shared across all the CreateJS libraries.  The classes are included in the minified
 * files of each library and are available on the createsjs namespace directly.
 *
 * <h4>Example</h4>
 *      myObject.addEventListener("change", createjs.proxy(myMethod, scope));
 *
 * @module CreateJS
 * @main CreateJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Contains properties and methods shared by all events for use with
 * {{#crossLink "EventDispatcher"}}{{/crossLink}}.
 * 
 * Note that Event objects are often reused, so you should never
 * rely on an event object's state outside of the call stack it was received in.
 * @class Event
 * @param {String} type The event type.
 * @param {Boolean} bubbles Indicates whether the event will bubble through the display list.
 * @param {Boolean} cancelable Indicates whether the default behaviour of this event can be cancelled.
 * @constructor
 **/
var Event = function(type, bubbles, cancelable) {
  this.initialize(type, bubbles, cancelable);
};
var p = Event.prototype;

// events:

// public properties:

	/**
	 * The type of event.
	 * @property type
	 * @type String
	 **/
	p.type = null;

	/**
	 * The object that generated an event.
	 * @property target
	 * @type Object
	 * @default null
	 * @readonly
	*/
	p.target = null;

	/**
	 * The current target that a bubbling event is being dispatched from. For non-bubbling events, this will
	 * always be the same as target. For example, if childObj.parent = parentObj, and a bubbling event
	 * is generated from childObj, then a listener on parentObj would receive the event with
	 * target=childObj (the original target) and currentTarget=parentObj (where the listener was added).
	 * @property currentTarget
	 * @type Object
	 * @default null
	 * @readonly
	*/
	p.currentTarget = null;

	/**
	 * For bubbling events, this indicates the current event phase:<OL>
	 * 	<LI> capture phase: starting from the top parent to the target</LI>
	 * 	<LI> at target phase: currently being dispatched from the target</LI>
	 * 	<LI> bubbling phase: from the target to the top parent</LI>
	 * </OL>
	 * @property eventPhase
	 * @type Number
	 * @default 0
	 * @readonly
	*/
	p.eventPhase = 0;

	/**
	 * Indicates whether the event will bubble through the display list.
	 * @property bubbles
	 * @type Boolean
	 * @default false
	 * @readonly
	*/
	p.bubbles = false;

	/**
	 * Indicates whether the default behaviour of this event can be cancelled via
	 * {{#crossLink "Event/preventDefault"}}{{/crossLink}}. This is set via the Event constructor.
	 * @property cancelable
	 * @type Boolean
	 * @default false
	 * @readonly
	*/
	p.cancelable = false;

	/**
	 * The epoch time at which this event was created.
	 * @property timeStamp
	 * @type Number
	 * @default 0
	 * @readonly
	*/
	p.timeStamp = 0;

	/**
	 * Indicates if {{#crossLink "Event/preventDefault"}}{{/crossLink}} has been called
	 * on this event.
	 * @property defaultPrevented
	 * @type Boolean
	 * @default false
	 * @readonly
	*/
	p.defaultPrevented = false;

	/**
	 * Indicates if {{#crossLink "Event/stopPropagation"}}{{/crossLink}} or
	 * {{#crossLink "Event/stopImmediatePropagation"}}{{/crossLink}} has been called on this event.
	 * @property propagationStopped
	 * @type Boolean
	 * @default false
	 * @readonly
	*/
	p.propagationStopped = false;

	/**
	 * Indicates if {{#crossLink "Event/stopImmediatePropagation"}}{{/crossLink}} has been called
	 * on this event.
	 * @property immediatePropagationStopped
	 * @type Boolean
	 * @default false
	 * @readonly
	*/
	p.immediatePropagationStopped = false;
	
	/**
	 * Indicates if {{#crossLink "Event/remove"}}{{/crossLink}} has been called on this event.
	 * @property removed
	 * @type Boolean
	 * @default false
	 * @readonly
	*/
	p.removed = false;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @param {String} type The event type.
	 * @param {Boolean} bubbles Indicates whether the event will bubble through the display list.
	 * @param {Boolean} cancelable Indicates whether the default behaviour of this event can be cancelled.
	 * @protected
	 **/
	p.initialize = function(type, bubbles, cancelable) {
		this.type = type;
		this.bubbles = bubbles;
		this.cancelable = cancelable;
		this.timeStamp = (new Date()).getTime();
	};

// public methods:

	/**
	 * Sets {{#crossLink "Event/defaultPrevented"}}{{/crossLink}} to true.
	 * Mirrors the DOM event standard.
	 * @method preventDefault
	 **/
	p.preventDefault = function() {
		this.defaultPrevented = true;
	};

	/**
	 * Sets {{#crossLink "Event/propagationStopped"}}{{/crossLink}} to true.
	 * Mirrors the DOM event standard.
	 * @method stopPropagation
	 **/
	p.stopPropagation = function() {
		this.propagationStopped = true;
	};

	/**
	 * Sets {{#crossLink "Event/propagationStopped"}}{{/crossLink}} and
	 * {{#crossLink "Event/immediatePropagationStopped"}}{{/crossLink}} to true.
	 * Mirrors the DOM event standard.
	 * @method stopImmediatePropagation
	 **/
	p.stopImmediatePropagation = function() {
		this.immediatePropagationStopped = this.propagationStopped = true;
	};
	
	/**
	 * Causes the active listener to be removed via removeEventListener();
	 * 
	 * 		myBtn.addEventListener("click", function(evt) {
	 * 			// do stuff...
	 * 			evt.remove(); // removes this listener.
	 * 		});
	 * 
	 * @method remove
	 **/
	p.remove = function() {
		this.removed = true;
	};
	
	/**
	 * Returns a clone of the Event instance.
	 * @method clone
	 * @return {Event} a clone of the Event instance.
	 **/
	p.clone = function() {
		return new Event(this.type, this.bubbles, this.cancelable);
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Event (type="+this.type+")]";
	};

createjs.Event = Event;
}());
/*
* EventDispatcher
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module CreateJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * EventDispatcher provides methods for managing queues of event listeners and dispatching events.
 *
 * You can either extend EventDispatcher or mix its methods into an existing prototype or instance by using the
 * EventDispatcher {{#crossLink "EventDispatcher/initialize"}}{{/crossLink}} method.
 * 
 * Together with the CreateJS Event class, EventDispatcher provides an extended event model that is based on the
 * DOM Level 2 event model, including addEventListener, removeEventListener, and dispatchEvent. It supports
 * bubbling / capture, preventDefault, stopPropagation, stopImmediatePropagation, and handleEvent.
 * 
 * EventDispatcher also exposes a {{#crossLink "EventDispatcher/on"}}{{/crossLink}} method, which makes it easier
 * to create scoped listeners, listeners that only run once, and listeners with associated arbitrary data. The 
 * {{#crossLink "EventDispatcher/off"}}{{/crossLink}} method is merely an alias to
 * {{#crossLink "EventDispatcher/removeEventListener"}}{{/crossLink}}.
 * 
 * Another addition to the DOM Level 2 model is the {{#crossLink "EventDispatcher/removeAllEventListeners"}}{{/crossLink}}
 * method, which can be used to listeners for all events, or listeners for a specific event. The Event object also 
 * includes a {{#crossLink "Event/remove"}}{{/crossLink}} method which removes the active listener.
 *
 * <h4>Example</h4>
 * Add EventDispatcher capabilities to the "MyClass" class.
 *
 *      EventDispatcher.initialize(MyClass.prototype);
 *
 * Add an event (see {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}}).
 *
 *      instance.addEventListener("eventName", handlerMethod);
 *      function handlerMethod(event) {
 *          console.log(event.target + " Was Clicked");
 *      }
 *
 * <b>Maintaining proper scope</b><br />
 * Scope (ie. "this") can be be a challenge with events. Using the {{#crossLink "EventDispatcher/on"}}{{/crossLink}}
 * method to subscribe to events simplifies this.
 *
 *      instance.addEventListener("click", function(event) {
 *          console.log(instance == this); // false, scope is ambiguous.
 *      });
 *      
 *      instance.on("click", function(event) {
 *          console.log(instance == this); // true, "on" uses dispatcher scope by default.
 *      });
 * 
 * If you want to use addEventListener instead, you may want to use function.bind() or a similar proxy to manage scope.
 *      
 *
 * @class EventDispatcher
 * @constructor
 **/
var EventDispatcher = function() {
/*	this.initialize(); */ // not needed.
};
var p = EventDispatcher.prototype;


	/**
	 * Static initializer to mix EventDispatcher methods into a target object or prototype.
	 * 
	 * 		EventDispatcher.initialize(MyClass.prototype); // add to the prototype of the class
	 * 		EventDispatcher.initialize(myObject); // add to a specific instance
	 * 
	 * @method initialize
	 * @static
	 * @param {Object} target The target object to inject EventDispatcher methods into. This can be an instance or a
	 * prototype.
	 **/
	EventDispatcher.initialize = function(target) {
		target.addEventListener = p.addEventListener;
		target.on = p.on;
		target.removeEventListener = target.off =  p.removeEventListener;
		target.removeAllEventListeners = p.removeAllEventListeners;
		target.hasEventListener = p.hasEventListener;
		target.dispatchEvent = p.dispatchEvent;
		target._dispatchEvent = p._dispatchEvent;
		target.willTrigger = p.willTrigger;
	};
	
// constructor:

// private properties:
	/**
	 * @protected
	 * @property _listeners
	 * @type Object
	 **/
	p._listeners = null;

	/**
	 * @protected
	 * @property _captureListeners
	 * @type Object
	 **/
	p._captureListeners = null;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	 **/
	p.initialize = function() {};

// public methods:
	/**
	 * Adds the specified event listener. Note that adding multiple listeners to the same function will result in
	 * multiple callbacks getting fired.
	 *
	 * <h4>Example</h4>
	 *
	 *      displayObject.addEventListener("click", handleClick);
	 *      function handleClick(event) {
	 *         // Click happened.
	 *      }
	 *
	 * @method addEventListener
	 * @param {String} type The string type of the event.
	 * @param {Function | Object} listener An object with a handleEvent method, or a function that will be called when
	 * the event is dispatched.
	 * @param {Boolean} [useCapture] For events that bubble, indicates whether to listen for the event in the capture or bubbling/target phase.
	 * @return {Function | Object} Returns the listener for chaining or assignment.
	 **/
	p.addEventListener = function(type, listener, useCapture) {
		var listeners;
		if (useCapture) {
			listeners = this._captureListeners = this._captureListeners||{};
		} else {
			listeners = this._listeners = this._listeners||{};
		}
		var arr = listeners[type];
		if (arr) { this.removeEventListener(type, listener, useCapture); }
		arr = listeners[type]; // remove may have deleted the array
		if (!arr) { listeners[type] = [listener];  }
		else { arr.push(listener); }
		return listener;
	};
	
	/**
	 * A shortcut method for using addEventListener that makes it easier to specify an execution scope, have a listener
	 * only run once, associate arbitrary data with the listener, and remove the listener.
	 * 
	 * This method works by creating an anonymous wrapper function and subscribing it with addEventListener.
	 * The created anonymous function is returned for use with .removeEventListener (or .off).
	 * 
	 * <h4>Example</h4>
	 * 
	 * 		var listener = myBtn.on("click", handleClick, null, false, {count:3});
	 * 		function handleClick(evt, data) {
	 * 			data.count -= 1;
	 * 			console.log(this == myBtn); // true - scope defaults to the dispatcher
	 * 			if (data.count == 0) {
	 * 				alert("clicked 3 times!");
	 * 				myBtn.off("click", listener);
	 * 				// alternately: evt.remove();
	 * 			}
	 * 		}
	 * 
	 * @method on
	 * @param {String} type The string type of the event.
	 * @param {Function | Object} listener An object with a handleEvent method, or a function that will be called when
	 * the event is dispatched.
	 * @param {Object} [scope] The scope to execute the listener in. Defaults to the dispatcher/currentTarget for function listeners, and to the listener itself for object listeners (ie. using handleEvent).
	 * @param {Boolean} [once=false] If true, the listener will remove itself after the first time it is triggered.
	 * @param {*} [data] Arbitrary data that will be included as the second parameter when the listener is called.
	 * @param {Boolean} [useCapture=false] For events that bubble, indicates whether to listen for the event in the capture or bubbling/target phase.
	 * @return {Function} Returns the anonymous function that was created and assigned as the listener. This is needed to remove the listener later using .removeEventListener.
	 **/
	p.on = function(type, listener, scope, once, data, useCapture) {
		if (listener.handleEvent) {
			scope = scope||listener;
			listener = listener.handleEvent;
		}
		scope = scope||this;
		return this.addEventListener(type, function(evt) {
				listener.call(scope, evt, data);
				once&&evt.remove();
			}, useCapture);
	};

	/**
	 * Removes the specified event listener.
	 *
	 * <b>Important Note:</b> that you must pass the exact function reference used when the event was added. If a proxy
	 * function, or function closure is used as the callback, the proxy/closure reference must be used - a new proxy or
	 * closure will not work.
	 *
	 * <h4>Example</h4>
	 *
	 *      displayObject.removeEventListener("click", handleClick);
	 *
	 * @method removeEventListener
	 * @param {String} type The string type of the event.
	 * @param {Function | Object} listener The listener function or object.
	 * @param {Boolean} [useCapture] For events that bubble, indicates whether to listen for the event in the capture or bubbling/target phase.
	 **/
	p.removeEventListener = function(type, listener, useCapture) {
		var listeners = useCapture ? this._captureListeners : this._listeners;
		if (!listeners) { return; }
		var arr = listeners[type];
		if (!arr) { return; }
		for (var i=0,l=arr.length; i<l; i++) {
			if (arr[i] == listener) {
				if (l==1) { delete(listeners[type]); } // allows for faster checks.
				else { arr.splice(i,1); }
				break;
			}
		}
	};
	
	/**
	 * A shortcut to the removeEventListener method, with the same parameters and return value. This is a companion to the
	 * .on method.
	 *
	 * @method off
	 * @param {String} type The string type of the event.
	 * @param {Function | Object} listener The listener function or object.
	 * @param {Boolean} [useCapture] For events that bubble, indicates whether to listen for the event in the capture or bubbling/target phase.
	 **/
	p.off = p.removeEventListener;

	/**
	 * Removes all listeners for the specified type, or all listeners of all types.
	 *
	 * <h4>Example</h4>
	 *
	 *      // Remove all listeners
	 *      displayObject.removeAllEventListeners();
	 *
	 *      // Remove all click listeners
	 *      displayObject.removeAllEventListeners("click");
	 *
	 * @method removeAllEventListeners
	 * @param {String} [type] The string type of the event. If omitted, all listeners for all types will be removed.
	 **/
	p.removeAllEventListeners = function(type) {
		if (!type) { this._listeners = this._captureListeners = null; }
		else {
			if (this._listeners) { delete(this._listeners[type]); }
			if (this._captureListeners) { delete(this._captureListeners[type]); }
		}
	};

	/**
	 * Dispatches the specified event to all listeners.
	 *
	 * <h4>Example</h4>
	 *
	 *      // Use a string event
	 *      this.dispatchEvent("complete");
	 *
	 *      // Use an Event instance
	 *      var event = new createjs.Event("progress");
	 *      this.dispatchEvent(event);
	 *
	 * @method dispatchEvent
	 * @param {Object | String | Event} eventObj An object with a "type" property, or a string type.
	 * While a generic object will work, it is recommended to use a CreateJS Event instance. If a string is used,
	 * dispatchEvent will construct an Event instance with the specified type.
	 * @param {Object} [target] The object to use as the target property of the event object. This will default to the
	 * dispatching object. <b>This parameter is deprecated and will be removed.</b>
	 * @return {Boolean} Returns the value of eventObj.defaultPrevented.
	 **/
	p.dispatchEvent = function(eventObj, target) {
		if (typeof eventObj == "string") {
			// won't bubble, so skip everything if there's no listeners:
			var listeners = this._listeners;
			if (!listeners || !listeners[eventObj]) { return false; }
			eventObj = new createjs.Event(eventObj);
		}
		// TODO: deprecated. Target param is deprecated, only use case is MouseEvent/mousemove, remove.
		eventObj.target = target||this;

		if (!eventObj.bubbles || !this.parent) {
			this._dispatchEvent(eventObj, 2);
		} else {
			var top=this, list=[top];
			while (top.parent) { list.push(top = top.parent); }
			var i, l=list.length;

			// capture & atTarget
			for (i=l-1; i>=0 && !eventObj.propagationStopped; i--) {
				list[i]._dispatchEvent(eventObj, 1+(i==0));
			}
			// bubbling
			for (i=1; i<l && !eventObj.propagationStopped; i++) {
				list[i]._dispatchEvent(eventObj, 3);
			}
		}
		return eventObj.defaultPrevented;
	};

	/**
	 * Indicates whether there is at least one listener for the specified event type.
	 * @method hasEventListener
	 * @param {String} type The string type of the event.
	 * @return {Boolean} Returns true if there is at least one listener for the specified event.
	 **/
	p.hasEventListener = function(type) {
		var listeners = this._listeners, captureListeners = this._captureListeners;
		return !!((listeners && listeners[type]) || (captureListeners && captureListeners[type]));
	};
	
	/**
	 * Indicates whether there is at least one listener for the specified event type on this object or any of its
	 * ancestors (parent, parent's parent, etc). A return value of true indicates that if a bubbling event of the
	 * specified type is dispatched from this object, it will trigger at least one listener.
	 * 
	 * This is similar to {{#crossLink "EventDispatcher/hasEventListener"}}{{/crossLink}}, but it searches the entire
	 * event flow for a listener, not just this object.
	 * @method willTrigger
	 * @param {String} type The string type of the event.
	 * @return {Boolean} Returns `true` if there is at least one listener for the specified event.
	 **/
	p.willTrigger = function(type) {
		var o = this;
		while (o) {
			if (o.hasEventListener(type)) { return true; }
			o = o.parent;
		}
		return false;
	};

	/**
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[EventDispatcher]";
	};

// private methods:
	/**
	 * @method _dispatchEvent
	 * @param {Object | String | Event} eventObj
	 * @param {Object} eventPhase
	 * @protected
	 **/
	p._dispatchEvent = function(eventObj, eventPhase) {
		var l, listeners = (eventPhase==1) ? this._captureListeners : this._listeners;
		if (eventObj && listeners) {
			var arr = listeners[eventObj.type];
			if (!arr||!(l=arr.length)) { return; }
			eventObj.currentTarget = this;
			eventObj.eventPhase = eventPhase;
			eventObj.removed = false;
			arr = arr.slice(); // to avoid issues with items being removed or added during the dispatch
			for (var i=0; i<l && !eventObj.immediatePropagationStopped; i++) {
				var o = arr[i];
				if (o.handleEvent) { o.handleEvent(eventObj); }
				else { o(eventObj); }
				if (eventObj.removed) {
					this.off(eventObj.type, o, eventPhase==1);
					eventObj.removed = false;
				}
			}
		}
	};


createjs.EventDispatcher = EventDispatcher;
}());
/*
* IndexOf
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
* 
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
* 
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
  * @module CreateJS
 */

// namespace:
this.createjs = this.createjs||{};

/**
 * @class Utility Methods
 */
(function() {
	"use strict";

	/*
	 * Employs Duff's Device to make a more performant implementation of indexOf.
	 * see http://jsperf.com/duffs-indexof/2
	 * #method indexOf
	 * @param {Array} array Array to search for searchElement
	 * @param searchElement Element to search array for.
	 * @return {Number} The position of the first occurrence of a specified value searchElement in the passed in array ar.
	 * @constructor
	 */
	/* replaced with simple for loop for now, perhaps will be researched further
	createjs.indexOf = function (ar, searchElement) {
		var l = ar.length;

		var n = (l * 0.125) ^ 0;	// 0.125 == 1/8, using multiplication because it's faster in some browsers	// ^0 floors result
		for (var i = 0; i < n; i++) {
			if(searchElement === ar[i*8])   { return (i*8);}
			if(searchElement === ar[i*8+1]) { return (i*8+1);}
			if(searchElement === ar[i*8+2]) { return (i*8+2);}
			if(searchElement === ar[i*8+3]) { return (i*8+3);}
			if(searchElement === ar[i*8+4]) { return (i*8+4);}
			if(searchElement === ar[i*8+5]) { return (i*8+5);}
			if(searchElement === ar[i*8+6]) { return (i*8+6);}
			if(searchElement === ar[i*8+7]) { return (i*8+7);}
		}

		var n = l % 8;
		for (var i = 0; i < n; i++) {
			if (searchElement === ar[l-n+i]) {
				return l-n+i;
			}
		}

		return -1;
	}
	*/

	/**
	 * Finds the first occurrence of a specified value searchElement in the passed in array, and returns the index of
	 * that value.  Returns -1 if value is not found.
	 *
	 *      var i = createjs.indexOf(myArray, myElementToFind);
	 *
	 * @method indexOf
	 * @param {Array} array Array to search for searchElement
	 * @param searchElement Element to find in array.
	 * @return {Number} The first index of searchElement in array.
	 */
	createjs.indexOf = function (array, searchElement){
		for (var i = 0,l=array.length; i < l; i++) {
			if (searchElement === array[i]) {
				return i;
			}
		}
		return -1;
	}

}());/*
* UID
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Global utility for generating sequential unique ID numbers. The UID class uses a static interface (ex. <code>UID.get()</code>)
 * and should not be instantiated.
 * @class UID
 * @static
 **/
var UID = function() {
	throw "UID cannot be instantiated";
}

	/**
	 * @property _nextID
	 * @type Number
	 * @protected
	 **/
	UID._nextID = 0;

	/**
	 * Returns the next unique id.
	 * @method get
	 * @return {Number} The next unique id
	 * @static
	 **/
	UID.get = function() {
		return UID._nextID++;
	}

createjs.UID = UID;
}());
/*
* Ticker
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

// constructor:
/**
 * The Ticker provides  a centralized tick or heartbeat broadcast at a set interval. Listeners can subscribe to the tick
 * event to be notified when a set time interval has elapsed.
 *
 * Note that the interval that the tick event is called is a target interval, and may be broadcast at a slower interval
 * during times of high CPU load. The Ticker class uses a static interface (ex. <code>Ticker.getPaused()</code>) and
 * should not be instantiated.
 *
 * <h4>Example</h4>
 *      createjs.Ticker.addEventListener("tick", handleTick);
 *      function handleTick(event) {
 *          // Actions carried out each frame
 *          if (!event.paused) {
 *              // Actions carried out when the Ticker is not paused.
 *          }
 *      }
 *
 * To update a stage every tick, the {{#crossLink "Stage"}}{{/crossLink}} instance can also be used as a listener, as
 * it will automatically update when it receives a tick event:
 *
 *      createjs.Ticker.addEventListener("tick", stage);
 *
 * @class Ticker
 * @uses EventDispatcher
 * @static
 **/
var Ticker = function() {
	throw "Ticker cannot be instantiated.";
};

// constants:
	/**
	 * In this mode, Ticker uses the requestAnimationFrame API, but attempts to synch the ticks to target framerate. It
	 * uses a simple heuristic that compares the time of the RAF return to the target time for the current frame and
	 * dispatches the tick when the time is within a certain threshold.
	 *
	 * This mode has a higher variance for time between frames than TIMEOUT, but does not require that content be time
	 * based as with RAF while gaining the benefits of that API (screen synch, background throttling).
	 *
	 * Variance is usually lowest for framerates that are a divisor of the RAF frequency. This is usually 60, so
	 * framerates of 10, 12, 15, 20, and 30 work well.
	 *
	 * Falls back on TIMEOUT if the requestAnimationFrame API is not supported.
	 * @property RAF_SYNCHED
	 * @static
	 * @type {String}
	 * @default "synched"
	 * @readonly
	 **/
	Ticker.RAF_SYNCHED = "synched";

	/**
	 * In this mode, Ticker passes through the requestAnimationFrame heartbeat, ignoring the target framerate completely.
	 * Because requestAnimationFrame frequency is not deterministic, any content using this mode should be time based.
	 * You can leverage {{#crossLink "Ticker/getTime"}}{{/crossLink}} and the tick event object's "delta" properties
	 * to make this easier.
	 *
	 * Falls back on TIMEOUT if the requestAnimationFrame API is not supported.
	 * @property RAF
	 * @static
	 * @type {String}
	 * @default "raf"
	 * @readonly
	 **/
	Ticker.RAF = "raf";

	/**
	 * In this mode, Ticker uses the setTimeout API. This provides predictable, adaptive frame timing, but does not
	 * provide the benefits of requestAnimationFrame (screen synch, background throttling).
	 * @property TIMEOUT
	 * @static
	 * @type {String}
	 * @default "timer"
	 * @readonly
	 **/
	Ticker.TIMEOUT = "timeout";

// events:

	/**
	 * Dispatched each tick. The event will be dispatched to each listener even when the Ticker has been paused using
	 * {{#crossLink "Ticker/setPaused"}}{{/crossLink}}.
	 *
	 * <h4>Example</h4>
	 *      createjs.Ticker.addEventListener("tick", handleTick);
	 *      function handleTick(event) {
	 *          console.log("Paused:", event.paused, event.delta);
	 *      }
	 *
	 * @event tick
	 * @param {Object} target The object that dispatched the event.
	 * @param {String} type The event type.
	 * @param {Boolean} paused Indicates whether the ticker is currently paused.
	 * @param {Number} delta The time elapsed in ms since the last tick.
	 * @param {Number} time The total time in ms since Ticker was initialized.
	 * @param {Number} runTime The total time in ms that Ticker was not paused since it was initialized. For example,
	 * 	you could determine the amount of time that the Ticker has been paused since initialization with time-runTime.
	 * @since 0.6.0
	 */

// public static properties:
	/**
	 * Deprecated in favour of {{#crossLink "Ticker/timingMode"}}{{/crossLink}}, and will be removed in a future version. If true, timingMode will
	 * use {{#crossLink "Ticker/RAF_SYNCHED"}}{{/crossLink}} by default.
	 * @deprecated Deprecated in favour of {{#crossLink "Ticker/timingMode"}}{{/crossLink}}.
	 * @property useRAF
	 * @static
	 * @type {Boolean}
	 * @default false
	 **/
	Ticker.useRAF = false;

	/**
	 * Specifies the timing api (setTimeout or requestAnimationFrame) and mode to use. See
	 * {{#crossLink "Ticker/TIMEOUT"}}{{/crossLink}}, {{#crossLink "Ticker/RAF"}}{{/crossLink}}, and
	 * {{#crossLink "Ticker/RAF_SYNCHED"}}{{/crossLink}} for mode details.
	 * @property timingMode
	 * @static
	 * @type {String}
	 * @default Ticker.TIMEOUT
	 **/
	Ticker.timingMode = null;

	/**
	 * Specifies a maximum value for the delta property in the tick event object. This is useful when building time
	 * based animations and systems to prevent issues caused by large time gaps caused by background tabs, system sleep,
	 * alert dialogs, or other blocking routines. Double the expected frame duration is often an effective value
	 * (ex. maxDelta=50 when running at 40fps).
	 * 
	 * This does not impact any other values (ex. time, runTime, etc), so you may experience issues if you enable maxDelta
	 * when using both delta and other values.
	 * 
	 * If 0, there is no maximum.
	 * @property maxDelta
	 * @static
	 * @type {number}
	 * @default 0
	 */
	Ticker.maxDelta = 0;

// mix-ins:
	// EventDispatcher methods:
	Ticker.removeEventListener = null;
	Ticker.removeAllEventListeners = null;
	Ticker.dispatchEvent = null;
	Ticker.hasEventListener = null;
	Ticker._listeners = null;
	createjs.EventDispatcher.initialize(Ticker); // inject EventDispatcher methods.
	Ticker._addEventListener = Ticker.addEventListener;
	Ticker.addEventListener = function() {
		!Ticker._inited&&Ticker.init();
		return Ticker._addEventListener.apply(Ticker, arguments);
	};

// private static properties:
	
	/** 
	 * @property _paused
	 * @type {Boolean}
	 * @protected
	 **/
	Ticker._paused = false;

	/**
	 * @property _inited
	 * @type {Boolean}
	 * @protected
	 **/
	Ticker._inited = false;

	/**
	 * @property _startTime
	 * @type {Number}
	 * @protected
	 **/
	Ticker._startTime = 0;

	/**
	 * @property _pausedTime
	 * @type {Number}
	 * @protected
	 **/
	Ticker._pausedTime=0;

	/**
	 * The number of ticks that have passed
	 * @property _ticks
	 * @type {Number}
	 * @protected
	 **/
	Ticker._ticks = 0;

	/**
	 * The number of ticks that have passed while Ticker has been paused
	 * @property _pausedTicks
	 * @type {Number}
	 * @protected
	 **/
	Ticker._pausedTicks = 0;

	/**
	 * @property _interval
	 * @type {Number}
	 * @protected
	 **/
	Ticker._interval = 50;

	/**
	 * @property _lastTime
	 * @type {Number}
	 * @protected
	 **/
	Ticker._lastTime = 0;

	/**
	 * @property _times
	 * @type {Array}
	 * @protected
	 **/
	Ticker._times = null;

	/**
	 * @property _tickTimes
	 * @type {Array}
	 * @protected
	 **/
	Ticker._tickTimes = null;

	/**
	 * Stores the timeout or requestAnimationFrame id.
	 * @property _timerId
	 * @type {Number}
	 * @protected
	 **/
	Ticker._timerId = null;
	
	/**
	 * True if currently using requestAnimationFrame, false if using setTimeout.
	 * @property _raf
	 * @type {Boolean}
	 * @protected
	 **/
	Ticker._raf = true;

// public static methods:
	
	/**
	 * Starts the tick. This is called automatically when the first listener is added.
	 * @method init
	 * @static
	 **/
	Ticker.init = function() {
		if (Ticker._inited) { return; }
		Ticker._inited = true;
		Ticker._times = [];
		Ticker._tickTimes = [];
		Ticker._startTime = Ticker._getTime();
		Ticker._times.push(Ticker._lastTime = 0);
		Ticker.setInterval(Ticker._interval);
	};
	
	/**
	 * Stops the Ticker and removes all listeners. Use init() to restart the Ticker.
	 * @method reset
	 * @static
	 **/
	Ticker.reset = function() {
		if (Ticker._raf) {
			var f = window.cancelAnimationFrame || window.webkitCancelAnimationFrame || window.mozCancelAnimationFrame || window.oCancelAnimationFrame || window.msCancelAnimationFrame;
			f&&f(Ticker._timerId);
		} else {
			clearTimeout(Ticker._timerId);
		}
		Ticker.removeAllEventListeners("tick");
	};
	
	/**
	 * Sets the target time (in milliseconds) between ticks. Default is 50 (20 FPS).
	 *
	 * Note actual time between ticks may be more than requested depending on CPU load.
	 * @method setInterval
	 * @static
	 * @param {Number} interval Time in milliseconds between ticks. Default value is 50.
	 **/
	Ticker.setInterval = function(interval) {
		Ticker._interval = interval;
		if (!Ticker._inited) { return; }
		Ticker._setupTick();
	};

	/**
	 * Returns the current target time between ticks, as set with {{#crossLink "Ticker/setInterval"}}{{/crossLink}}.
	 * @method getInterval
	 * @static
	 * @return {Number} The current target interval in milliseconds between tick events.
	 **/
	Ticker.getInterval = function() {
		return Ticker._interval;
	};

	/**
	 * Sets the target frame rate in frames per second (FPS). For example, with an interval of 40, <code>getFPS()</code>
	 * will return 25 (1000ms per second divided by 40 ms per tick = 25fps).
	 * @method setFPS
	 * @static
	 * @param {Number} value Target number of ticks broadcast per second.
	 **/
	Ticker.setFPS = function(value) {
		Ticker.setInterval(1000/value);
	};

	/**
	 * Returns the target frame rate in frames per second (FPS). For example, with an interval of 40, <code>getFPS()</code>
	 * will return 25 (1000ms per second divided by 40 ms per tick = 25fps).
	 * @method getFPS
	 * @static
	 * @return {Number} The current target number of frames / ticks broadcast per second.
	 **/
	Ticker.getFPS = function() {
		return 1000/Ticker._interval;
	};

	/**
	 * Returns the average time spent within a tick. This can vary significantly from the value provided by getMeasuredFPS
	 * because it only measures the time spent within the tick execution stack. 
	 * 
	 * Example 1: With a target FPS of 20, getMeasuredFPS() returns 20fps, which indicates an average of 50ms between 
	 * the end of one tick and the end of the next. However, getMeasuredTickTime() returns 15ms. This indicates that 
	 * there may be up to 35ms of "idle" time between the end of one tick and the start of the next.
	 *
	 * Example 2: With a target FPS of 30, getFPS() returns 10fps, which indicates an average of 100ms between the end of
	 * one tick and the end of the next. However, getMeasuredTickTime() returns 20ms. This would indicate that something
	 * other than the tick is using ~80ms (another script, DOM rendering, etc).
	 * @method getMeasuredTickTime
	 * @static
	 * @param {Number} [ticks] The number of previous ticks over which to measure the average time spent in a tick.
	 * Defaults to the number of ticks per second. To get only the last tick's time, pass in 1.
	 * @return {Number} The average time spent in a tick in milliseconds.
	 **/
	Ticker.getMeasuredTickTime = function(ticks) {
		var ttl=0, times=Ticker._tickTimes;
		if (times.length < 1) { return -1; }

		// by default, calculate average for the past ~1 second:
		ticks = Math.min(times.length, ticks||(Ticker.getFPS()|0));
		for (var i=0; i<ticks; i++) { ttl += times[i]; }
		return ttl/ticks;
	};

	/**
	 * Returns the actual frames / ticks per second.
	 * @method getMeasuredFPS
	 * @static
	 * @param {Number} [ticks] The number of previous ticks over which to measure the actual frames / ticks per second.
	 * Defaults to the number of ticks per second.
	 * @return {Number} The actual frames / ticks per second. Depending on performance, this may differ
	 * from the target frames per second.
	 **/
	Ticker.getMeasuredFPS = function(ticks) {
		var times = Ticker._times;
		if (times.length < 2) { return -1; }

		// by default, calculate fps for the past ~1 second:
		ticks = Math.min(times.length-1, ticks||(Ticker.getFPS()|0));
		return 1000/((times[0]-times[ticks])/ticks);
	};

	/**
	 * Changes the "paused" state of the Ticker, which can be retrieved by the {{#crossLink "Ticker/getPaused"}}{{/crossLink}}
	 * method, and is passed as the "paused" property of the <code>tick</code> event. When the ticker is paused, all
	 * listeners will still receive a tick event, but the <code>paused</code> property will be false.
	 *
	 * Note that in EaselJS v0.5.0 and earlier, "pauseable" listeners would <strong>not</strong> receive the tick
	 * callback when Ticker was paused. This is no longer the case.
	 *
	 * <h4>Example</h4>
	 *      createjs.Ticker.addEventListener("tick", handleTick);
	 *      createjs.Ticker.setPaused(true);
	 *      function handleTick(event) {
	 *          console.log("Paused:", event.paused, createjs.Ticker.getPaused());
	 *      }
	 *
	 * @method setPaused
	 * @static
	 * @param {Boolean} value Indicates whether to pause (true) or unpause (false) Ticker.
	 **/
	Ticker.setPaused = function(value) {
		Ticker._paused = value;
	};

	/**
	 * Returns a boolean indicating whether Ticker is currently paused, as set with {{#crossLink "Ticker/setPaused"}}{{/crossLink}}.
	 * When the ticker is paused, all listeners will still receive a tick event, but this value will be false.
	 *
	 * Note that in EaselJS v0.5.0 and earlier, "pauseable" listeners would <strong>not</strong> receive the tick
	 * callback when Ticker was paused. This is no longer the case.
	 *
	 * <h4>Example</h4>
	 *      createjs.Ticker.addEventListener("tick", handleTick);
	 *      createjs.Ticker.setPaused(true);
	 *      function handleTick(event) {
	 *          console.log("Paused:", createjs.Ticker.getPaused());
	 *      }
	 *
	 * @method getPaused
	 * @static
	 * @return {Boolean} Whether the Ticker is currently paused.
	 **/
	Ticker.getPaused = function() {
		return Ticker._paused;
	};

	/**
	 * Returns the number of milliseconds that have elapsed since Ticker was initialized. For example, you could use
	 * this in a time synchronized animation to determine the exact amount of time that has elapsed.
	 * @method getTime
	 * @static
	 * @param {Boolean} [runTime=false] If true only time elapsed while Ticker was not paused will be returned.
	 * If false, the value returned will be total time elapsed since the first tick event listener was added.
	 * @return {Number} Number of milliseconds that have elapsed since Ticker was initialized.
	 **/
	Ticker.getTime = function(runTime) {
		return Ticker._getTime() - Ticker._startTime - (runTime ? Ticker._pausedTime : 0);
	};

	/**
	 * Similar to getTime(), but returns the time included with the current (or most recent) tick event object.
	 * @method getEventTime
	 * @param runTime {Boolean} [runTime=false] If true, the runTime property will be returned instead of time.
	 * @returns {number} The time or runTime property from the most recent tick event.
	 */
	Ticker.getEventTime = function(runTime) {
		return (Ticker._lastTime || Ticker._startTime) - (runTime ? Ticker._pausedTime : 0);
	};
	
	/**
	 * Returns the number of ticks that have been broadcast by Ticker.
	 * @method getTicks
	 * @static
	 * @param {Boolean} pauseable Indicates whether to include ticks that would have been broadcast
	 * while Ticker was paused. If true only tick events broadcast while Ticker is not paused will be returned.
	 * If false, tick events that would have been broadcast while Ticker was paused will be included in the return
	 * value. The default value is false.
	 * @return {Number} of ticks that have been broadcast.
	 **/
	Ticker.getTicks = function(pauseable) {
		return  Ticker._ticks - (pauseable ?Ticker._pausedTicks : 0);
	};

// private static methods:
	/**
	 * @method _handleSynch
	 * @static
	 * @protected
	 **/
	Ticker._handleSynch = function() {
		var time = Ticker._getTime() - Ticker._startTime;
		Ticker._timerId = null;
		Ticker._setupTick();

		// run if enough time has elapsed, with a little bit of flexibility to be early:
		if (time - Ticker._lastTime >= (Ticker._interval-1)*0.97) {
			Ticker._tick();
		}
	};

	/**
	 * @method _handleRAF
	 * @static
	 * @protected
	 **/
	Ticker._handleRAF = function() {
		Ticker._timerId = null;
		Ticker._setupTick();
		Ticker._tick();
	};

	/**
	 * @method _handleTimeout
	 * @static
	 * @protected
	 **/
	Ticker._handleTimeout = function() {
		Ticker._timerId = null;
		Ticker._setupTick();
		Ticker._tick();
	};

	/**
	 * @method _setupTick
	 * @static
	 * @protected
	 **/
	Ticker._setupTick = function() {
		if (Ticker._timerId != null) { return; } // avoid duplicates

		var mode = Ticker.timingMode||(Ticker.useRAF&&Ticker.RAF_SYNCHED);
		if (mode == Ticker.RAF_SYNCHED || mode == Ticker.RAF) {
			var f = window.requestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame;
			if (f) {
				Ticker._timerId = f(mode == Ticker.RAF ? Ticker._handleRAF : Ticker._handleSynch);
				Ticker._raf = true;
				return;
			}
		}
		Ticker._raf = false;
		Ticker._timerId = setTimeout(Ticker._handleTimeout, Ticker._interval);
	};

	/**
	 * @method _tick
	 * @static
	 * @protected
	 **/
	Ticker._tick = function() {
		var time = Ticker._getTime()-Ticker._startTime;
		var elapsedTime = time-Ticker._lastTime;
		var paused = Ticker._paused;
		
		Ticker._ticks++;
		if (paused) {
			Ticker._pausedTicks++;
			Ticker._pausedTime += elapsedTime;
		}
		Ticker._lastTime = time;
		
		if (Ticker.hasEventListener("tick")) {
			var event = new createjs.Event("tick");
			var maxDelta = Ticker.maxDelta;
			event.delta = (maxDelta && elapsedTime > maxDelta) ? maxDelta : elapsedTime;
			event.paused = paused;
			event.time = time;
			event.runTime = time-Ticker._pausedTime;
			Ticker.dispatchEvent(event);
		}
		
		Ticker._tickTimes.unshift(Ticker._getTime()-time);
		while (Ticker._tickTimes.length > 100) { Ticker._tickTimes.pop(); }

		Ticker._times.unshift(time);
		while (Ticker._times.length > 100) { Ticker._times.pop(); }
	};

	/**
	 * @method _getTime
	 * @static
	 * @protected
	 **/
	var now = window.performance && (performance.now || performance.mozNow || performance.msNow || performance.oNow || performance.webkitNow);
	Ticker._getTime = function() {
		return (now&&now.call(performance))||(new Date().getTime());
	};

createjs.Ticker = Ticker;
}());
/*
* MouseEvent
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

// TODO: deprecated. @uses EventDispatcher
/**
 * Passed as the parameter to all mouse/pointer/touch related events. For a listing of mouse events and their properties,
 * see the {{#crossLink "DisplayObject"}}{{/crossLink}} and {{#crossLink "Stage"}}{{/crossLink}} event listings.
 * @class MouseEvent
 * @param {String} type The event type.
 * @param {Boolean} bubbles Indicates whether the event will bubble through the display list.
 * @param {Boolean} cancelable Indicates whether the default behaviour of this event can be cancelled.
 * @param {Number} stageX The normalized x position relative to the stage.
 * @param {Number} stageY The normalized y position relative to the stage.
 * @param {MouseEvent} nativeEvent The native DOM event related to this mouse event.
 * @param {Number} pointerID The unique id for the pointer.
 * @param {Boolean} primary Indicates whether this is the primary pointer in a multitouch environment.
 * @param {Number} rawX The raw x position relative to the stage.
 * @param {Number} rawY The raw y position relative to the stage.
 * @extends Event
 * @uses EventDispatcher
 * @constructor
 **/
var MouseEvent = function(type, bubbles, cancelable, stageX, stageY, nativeEvent, pointerID, primary, rawX, rawY) {
	this.initialize(type, bubbles, cancelable, stageX, stageY, nativeEvent, pointerID, primary, rawX, rawY);
};
var p = MouseEvent.prototype = new createjs.Event();

// events:

	/**
	 * For MouseEvent objects of type "mousedown", mousemove events will be dispatched from the event object until the
	 * user releases the mouse anywhere. This enables you to listen to mouse move interactions for the duration of a
	 * press, which can be very useful for operations such as drag and drop.
	 *
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class description for more information on mouse events.
	 * @event mousemove
	 * @since 0.6.0
	 * @deprecated In favour of the DisplayObject "pressmove" event.
	 */

	/**
	 * For MouseEvent objects of type "mousedown", a mouseup event will be dispatched from the event object when the
	 * user releases the mouse anywhere. This enables you to listen for a corresponding mouse up from a specific press,
	 * which can be very useful for operations such as drag and drop.
	 *
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class description for more information on mouse events.
	 * @event mouseup
	 * @since 0.6.0
	 * @deprecated In favour of the DisplayObject "pressup" event.
	 */

// public properties:

	/**
	 * The normalized x position on the stage. This will always be within the range 0 to stage width.
	 * @property stageX
	 * @type Number
	*/
	p.stageX = 0;

	/**
	 * The normalized y position on the stage. This will always be within the range 0 to stage height.
	 * @property stageY
	 * @type Number
	 **/
	p.stageY = 0;

	/**
	 * The raw x position relative to the stage. Normally this will be the same as the stageX value, unless
	 * stage.mouseMoveOutside is true and the pointer is outside of the stage bounds.
	 * @property rawX
	 * @type Number
	*/
	p.rawX = 0;

	/**
	 * The raw y position relative to the stage. Normally this will be the same as the stageY value, unless
	 * stage.mouseMoveOutside is true and the pointer is outside of the stage bounds.
	 * @property rawY
	 * @type Number
	*/
	p.rawY = 0;

	/**
	 * The native MouseEvent generated by the browser. The properties and API for this
	 * event may differ between browsers. This property will be null if the
	 * EaselJS property was not directly generated from a native MouseEvent.
	 * @property nativeEvent
	 * @type HtmlMouseEvent
	 * @default null
	 **/
	p.nativeEvent = null;

	// TODO: deprecated:
	/**
	 * REMOVED. Use the {{#crossLink "DisplayObject"}}{{/crossLink}} {{#crossLink "DisplayObject/pressmove:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseMove
	 * @type Function
	 * @deprecated Use the DisplayObject "pressmove" event.
	 */
	/**
	 * REMOVED. Use the {{#crossLink "DisplayObject"}}{{/crossLink}} {{#crossLink "DisplayObject/pressup:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseUp
	 * @type Function
	 * @deprecated Use the DisplayObject "pressup" event.
	 */

	/**
	 * The unique id for the pointer (touch point or cursor). This will be either -1 for the mouse, or the system
	 * supplied id value.
	 * @property pointerID
	 * @type {Number}
	 */
	p.pointerID = 0;

	/**
	 * Indicates whether this is the primary pointer in a multitouch environment. This will always be true for the mouse.
	 * For touch pointers, the first pointer in the current stack will be considered the primary pointer.
	 * @property primary
	 * @type {Boolean}
	 */
	p.primary = false;

// mix-ins:
	// EventDispatcher methods:
	// TODO: deprecated:
	p.addEventListener = null;
	p.removeEventListener = null;
	p.removeAllEventListeners = null;
	p.dispatchEvent = null;
	p.hasEventListener = null;
	p._listeners = null;
	createjs.EventDispatcher.initialize(p); // inject EventDispatcher methods.
	
// getter / setters:
	/**
	 * Returns the x position of the mouse in the local coordinate system of the current target (ie. the dispatcher).
	 * @property localX
	 * @type {Number}
	 * @readonly
	 */
	p._get_localX = function() {
		return this.currentTarget.globalToLocal(this.rawX, this.rawY).x;
	};
	
	/**
	 * Returns the y position of the mouse in the local coordinate system of the current target (ie. the dispatcher).
	 * @property localY
	 * @type {Number}
	 * @readonly
	 */
	p._get_localY = function() {
		return this.currentTarget.globalToLocal(this.rawX, this.rawY).y;
	};
	
	try {
		Object.defineProperties(p, {
			localX: { get: p._get_localX },
			localY: { get: p._get_localY }
		});
	} catch (e) {} // TODO: use Log

// constructor:
	/**
	 * @property Event_initialize
	 * @private
	 * @type Function
	 **/
	p.Event_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @param {String} type The event type.
	 * @param {Boolean} bubbles Indicates whether the event will bubble through the display list.
	 * @param {Boolean} cancelable Indicates whether the default behaviour of this event can be cancelled.
	 * @param {Number} stageX The normalized x position relative to the stage.
	 * @param {Number} stageY The normalized y position relative to the stage.
	 * @param {MouseEvent} nativeEvent The native DOM event related to this mouse event.
	 * @param {Number} pointerID The unique id for the pointer.
	 * @param {Boolean} primary Indicates whether this is the primary pointer in a multitouch environment.
	 * @param {Number} rawX The raw x position relative to the stage.
	 * @param {Number} rawY The raw y position relative to the stage.
	 * @protected
	 **/
	p.initialize = function(type, bubbles, cancelable, stageX, stageY, nativeEvent, pointerID, primary, rawX, rawY) {
		this.Event_initialize(type, bubbles, cancelable);
		this.stageX = stageX;
		this.stageY = stageY;
		this.nativeEvent = nativeEvent;
		this.pointerID = pointerID;
		this.primary = primary;
		this.rawX = (rawX==null)?stageX:rawX;
		this.rawY = (rawY==null)?stageY:rawY;
	};

// public methods:
	/**
	 * Returns a clone of the MouseEvent instance.
	 * @method clone
	 * @return {MouseEvent} a clone of the MouseEvent instance.
	 **/
	p.clone = function() {
		return new MouseEvent(this.type, this.bubbles, this.cancelable, this.stageX, this.stageY, this.target, this.nativeEvent, this.pointerID, this.primary, this.rawX, this.rawY);
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[MouseEvent (type="+this.type+" stageX="+this.stageX+" stageY="+this.stageY+")]";
	};

createjs.MouseEvent = MouseEvent;
}());
/*
* Matrix2D
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Represents an affine transformation matrix, and provides tools for constructing and concatenating matrixes.
 * @class Matrix2D
 * @param {Number} [a=1] Specifies the a property for the new matrix.
 * @param {Number} [b=0] Specifies the b property for the new matrix.
 * @param {Number} [c=0] Specifies the c property for the new matrix.
 * @param {Number} [d=1] Specifies the d property for the new matrix.
 * @param {Number} [tx=0] Specifies the tx property for the new matrix.
 * @param {Number} [ty=0] Specifies the ty property for the new matrix.
 * @constructor
 **/
var Matrix2D = function(a, b, c, d, tx, ty) {
  this.initialize(a, b, c, d, tx, ty);
};
var p = Matrix2D.prototype;

// static public properties:

	/**
	 * An identity matrix, representing a null transformation.
	 * @property identity
	 * @static
	 * @type Matrix2D
	 * @readonly
	 **/
	Matrix2D.identity = null; // set at bottom of class definition.

	/**
	 * Multiplier for converting degrees to radians. Used internally by Matrix2D.
	 * @property DEG_TO_RAD
	 * @static
	 * @final
	 * @type Number
	 * @readonly
	 **/
	Matrix2D.DEG_TO_RAD = Math.PI/180;


// public properties:
	/**
	 * Position (0, 0) in a 3x3 affine transformation matrix.
	 * @property a
	 * @type Number
	 **/
	p.a = 1;

	/**
	 * Position (0, 1) in a 3x3 affine transformation matrix.
	 * @property b
	 * @type Number
	 **/
	p.b = 0;

	/**
	 * Position (1, 0) in a 3x3 affine transformation matrix.
	 * @property c
	 * @type Number
	 **/
	p.c = 0;

	/**
	 * Position (1, 1) in a 3x3 affine transformation matrix.
	 * @property d
	 * @type Number
	 **/
	p.d = 1;

	/**
	 * Position (2, 0) in a 3x3 affine transformation matrix.
	 * @property tx
	 * @type Number
	 **/
	p.tx = 0;

	/**
	 * Position (2, 1) in a 3x3 affine transformation matrix.
	 * @property ty
	 * @type Number
	 **/
	p.ty = 0;

	/**
	 * Property representing the alpha that will be applied to a display object. This is not part of matrix
	 * operations, but is used for operations like getConcatenatedMatrix to provide concatenated alpha values.
	 * @property alpha
	 * @type Number
	 **/
	p.alpha = 1;

	/**
	 * Property representing the shadow that will be applied to a display object. This is not part of matrix
	 * operations, but is used for operations like getConcatenatedMatrix to provide concatenated shadow values.
	 * @property shadow
	 * @type Shadow
	 **/
	p.shadow  = null;

	/**
	 * Property representing the compositeOperation that will be applied to a display object. This is not part of
	 * matrix operations, but is used for operations like getConcatenatedMatrix to provide concatenated
	 * compositeOperation values. You can find a list of valid composite operations at:
	 * <a href="https://developer.mozilla.org/en/Canvas_tutorial/Compositing">https://developer.mozilla.org/en/Canvas_tutorial/Compositing</a>
	 * @property compositeOperation
	 * @type String
	 **/
	p.compositeOperation = null;

// constructor:
	/**
	 * Initialization method. Can also be used to reinitialize the instance.
	 * @method initialize
	 * @param {Number} [a=1] Specifies the a property for the new matrix.
	 * @param {Number} [b=0] Specifies the b property for the new matrix.
	 * @param {Number} [c=0] Specifies the c property for the new matrix.
	 * @param {Number} [d=1] Specifies the d property for the new matrix.
	 * @param {Number} [tx=0] Specifies the tx property for the new matrix.
	 * @param {Number} [ty=0] Specifies the ty property for the new matrix.
	 * @return {Matrix2D} This instance. Useful for chaining method calls.
	*/
	p.initialize = function(a, b, c, d, tx, ty) {
		this.a = (a == null) ? 1 : a;
		this.b = b || 0;
		this.c = c || 0;
		this.d = (d == null) ? 1 : d;
		this.tx = tx || 0;
		this.ty = ty || 0;
		return this;
	};

// public methods:
	/**
	 * Concatenates the specified matrix properties with this matrix. All parameters are required.
	 * @method prepend
	 * @param {Number} a
	 * @param {Number} b
	 * @param {Number} c
	 * @param {Number} d
	 * @param {Number} tx
	 * @param {Number} ty
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.prepend = function(a, b, c, d, tx, ty) {
		var tx1 = this.tx;
		if (a != 1 || b != 0 || c != 0 || d != 1) {
			var a1 = this.a;
			var c1 = this.c;
			this.a  = a1*a+this.b*c;
			this.b  = a1*b+this.b*d;
			this.c  = c1*a+this.d*c;
			this.d  = c1*b+this.d*d;
		}
		this.tx = tx1*a+this.ty*c+tx;
		this.ty = tx1*b+this.ty*d+ty;
		return this;
	};

	/**
	 * Appends the specified matrix properties with this matrix. All parameters are required.
	 * @method append
	 * @param {Number} a
	 * @param {Number} b
	 * @param {Number} c
	 * @param {Number} d
	 * @param {Number} tx
	 * @param {Number} ty
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.append = function(a, b, c, d, tx, ty) {
		var a1 = this.a;
		var b1 = this.b;
		var c1 = this.c;
		var d1 = this.d;

		this.a  = a*a1+b*c1;
		this.b  = a*b1+b*d1;
		this.c  = c*a1+d*c1;
		this.d  = c*b1+d*d1;
		this.tx = tx*a1+ty*c1+this.tx;
		this.ty = tx*b1+ty*d1+this.ty;
		return this;
	};

	/**
	 * Prepends the specified matrix with this matrix.
	 * @method prependMatrix
	 * @param {Matrix2D} matrix
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.prependMatrix = function(matrix) {
		this.prepend(matrix.a, matrix.b, matrix.c, matrix.d, matrix.tx, matrix.ty);
		this.prependProperties(matrix.alpha, matrix.shadow,  matrix.compositeOperation);
		return this;
	};

	/**
	 * Appends the specified matrix with this matrix.
	 * @method appendMatrix
	 * @param {Matrix2D} matrix
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.appendMatrix = function(matrix) {
		this.append(matrix.a, matrix.b, matrix.c, matrix.d, matrix.tx, matrix.ty);
		this.appendProperties(matrix.alpha, matrix.shadow,  matrix.compositeOperation);
		return this;
	};

	/**
	 * Generates matrix properties from the specified display object transform properties, and prepends them with this matrix.
	 * For example, you can use this to generate a matrix from a display object: var mtx = new Matrix2D();
	 * mtx.prependTransform(o.x, o.y, o.scaleX, o.scaleY, o.rotation);
	 * @method prependTransform
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Number} scaleX
	 * @param {Number} scaleY
	 * @param {Number} rotation
	 * @param {Number} skewX
	 * @param {Number} skewY
	 * @param {Number} regX Optional.
	 * @param {Number} regY Optional.
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.prependTransform = function(x, y, scaleX, scaleY, rotation, skewX, skewY, regX, regY) {
		if (rotation%360) {
			var r = rotation*Matrix2D.DEG_TO_RAD;
			var cos = Math.cos(r);
			var sin = Math.sin(r);
		} else {
			cos = 1;
			sin = 0;
		}

		if (regX || regY) {
			// append the registration offset:
			this.tx -= regX; this.ty -= regY;
		}
		if (skewX || skewY) {
			// TODO: can this be combined into a single prepend operation?
			skewX *= Matrix2D.DEG_TO_RAD;
			skewY *= Matrix2D.DEG_TO_RAD;
			this.prepend(cos*scaleX, sin*scaleX, -sin*scaleY, cos*scaleY, 0, 0);
			this.prepend(Math.cos(skewY), Math.sin(skewY), -Math.sin(skewX), Math.cos(skewX), x, y);
		} else {
			this.prepend(cos*scaleX, sin*scaleX, -sin*scaleY, cos*scaleY, x, y);
		}
		return this;
	};

	/**
	 * Generates matrix properties from the specified display object transform properties, and appends them with this matrix.
	 * For example, you can use this to generate a matrix from a display object: var mtx = new Matrix2D();
	 * mtx.appendTransform(o.x, o.y, o.scaleX, o.scaleY, o.rotation);
	 * @method appendTransform
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Number} scaleX
	 * @param {Number} scaleY
	 * @param {Number} rotation
	 * @param {Number} skewX
	 * @param {Number} skewY
	 * @param {Number} regX Optional.
	 * @param {Number} regY Optional.
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.appendTransform = function(x, y, scaleX, scaleY, rotation, skewX, skewY, regX, regY) {
		if (rotation%360) {
			var r = rotation*Matrix2D.DEG_TO_RAD;
			var cos = Math.cos(r);
			var sin = Math.sin(r);
		} else {
			cos = 1;
			sin = 0;
		}

		if (skewX || skewY) {
			// TODO: can this be combined into a single append?
			skewX *= Matrix2D.DEG_TO_RAD;
			skewY *= Matrix2D.DEG_TO_RAD;
			this.append(Math.cos(skewY), Math.sin(skewY), -Math.sin(skewX), Math.cos(skewX), x, y);
			this.append(cos*scaleX, sin*scaleX, -sin*scaleY, cos*scaleY, 0, 0);
		} else {
			this.append(cos*scaleX, sin*scaleX, -sin*scaleY, cos*scaleY, x, y);
		}

		if (regX || regY) {
			// prepend the registration offset:
			this.tx -= regX*this.a+regY*this.c; 
			this.ty -= regX*this.b+regY*this.d;
		}
		return this;
	};

	/**
	 * Applies a rotation transformation to the matrix.
	 * @method rotate
	 * @param {Number} angle The angle in radians. To use degrees, multiply by <code>Math.PI/180</code>.
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.rotate = function(angle) {
		var cos = Math.cos(angle);
		var sin = Math.sin(angle);

		var a1 = this.a;
		var c1 = this.c;
		var tx1 = this.tx;

		this.a = a1*cos-this.b*sin;
		this.b = a1*sin+this.b*cos;
		this.c = c1*cos-this.d*sin;
		this.d = c1*sin+this.d*cos;
		this.tx = tx1*cos-this.ty*sin;
		this.ty = tx1*sin+this.ty*cos;
		return this;
	};

	/**
	 * Applies a skew transformation to the matrix.
	 * @method skew
	 * @param {Number} skewX The amount to skew horizontally in degrees.
	 * @param {Number} skewY The amount to skew vertically in degrees.
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	*/
	p.skew = function(skewX, skewY) {
		skewX = skewX*Matrix2D.DEG_TO_RAD;
		skewY = skewY*Matrix2D.DEG_TO_RAD;
		this.append(Math.cos(skewY), Math.sin(skewY), -Math.sin(skewX), Math.cos(skewX), 0, 0);
		return this;
	};

	/**
	 * Applies a scale transformation to the matrix.
	 * @method scale
	 * @param {Number} x The amount to scale horizontally
	 * @param {Number} y The amount to scale vertically
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.scale = function(x, y) {
		this.a *= x;
		this.d *= y;
		this.c *= x;
		this.b *= y;
		this.tx *= x;
		this.ty *= y;
		return this;
	};

	/**
	 * Translates the matrix on the x and y axes.
	 * @method translate
	 * @param {Number} x
	 * @param {Number} y
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.translate = function(x, y) {
		this.tx += x;
		this.ty += y;
		return this;
	};

	/**
	 * Sets the properties of the matrix to those of an identity matrix (one that applies a null transformation).
	 * @method identity
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.identity = function() {
		this.alpha = this.a = this.d = 1;
		this.b = this.c = this.tx = this.ty = 0;
		this.shadow = this.compositeOperation = null;
		return this;
	};

	/**
	 * Inverts the matrix, causing it to perform the opposite transformation.
	 * @method invert
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	 **/
	p.invert = function() {
		var a1 = this.a;
		var b1 = this.b;
		var c1 = this.c;
		var d1 = this.d;
		var tx1 = this.tx;
		var n = a1*d1-b1*c1;

		this.a = d1/n;
		this.b = -b1/n;
		this.c = -c1/n;
		this.d = a1/n;
		this.tx = (c1*this.ty-d1*tx1)/n;
		this.ty = -(a1*this.ty-b1*tx1)/n;
		return this;
	};

	/**
	 * Returns true if the matrix is an identity matrix.
	 * @method isIdentity
	 * @return {Boolean}
	 **/
	p.isIdentity = function() {
		return this.tx == 0 && this.ty == 0 && this.a == 1 && this.b == 0 && this.c == 0 && this.d == 1;
	};

	/**
	 * Transforms a point according to this matrix.
	 * @method transformPoint
	 * @param {Number} x The x component of the point to transform.
	 * @param {Number} y The y component of the point to transform.
	 * @param {Point | Object} [pt] An object to copy the result into. If omitted a generic object with x/y properties will be returned.
	 * @return {Point} This matrix. Useful for chaining method calls.
	 **/
	p.transformPoint = function(x, y, pt) {
		pt = pt||{};
		pt.x = x*this.a+y*this.c+this.tx;
		pt.y = x*this.b+y*this.d+this.ty;
		return pt;
	};

	/**
	 * Decomposes the matrix into transform properties (x, y, scaleX, scaleY, and rotation). Note that this these values
	 * may not match the transform properties you used to generate the matrix, though they will produce the same visual
	 * results.
	 * @method decompose
	 * @param {Object} target The object to apply the transform properties to. If null, then a new object will be returned.
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	*/
	p.decompose = function(target) {
		// TODO: it would be nice to be able to solve for whether the matrix can be decomposed into only scale/rotation
		// even when scale is negative
		if (target == null) { target = {}; }
		target.x = this.tx;
		target.y = this.ty;
		target.scaleX = Math.sqrt(this.a * this.a + this.b * this.b);
		target.scaleY = Math.sqrt(this.c * this.c + this.d * this.d);

		var skewX = Math.atan2(-this.c, this.d);
		var skewY = Math.atan2(this.b, this.a);

		if (skewX == skewY) {
			target.rotation = skewY/Matrix2D.DEG_TO_RAD;
			if (this.a < 0 && this.d >= 0) {
				target.rotation += (target.rotation <= 0) ? 180 : -180;
			}
			target.skewX = target.skewY = 0;
		} else {
			target.skewX = skewX/Matrix2D.DEG_TO_RAD;
			target.skewY = skewY/Matrix2D.DEG_TO_RAD;
		}
		return target;
	};

	/**
	 * Reinitializes all matrix properties to those specified.
	 * @method reinitialize
	 * @param {Number} [a=1] Specifies the a property for the new matrix.
	 * @param {Number} [b=0] Specifies the b property for the new matrix.
	 * @param {Number} [c=0] Specifies the c property for the new matrix.
	 * @param {Number} [d=1] Specifies the d property for the new matrix.
	 * @param {Number} [tx=0] Specifies the tx property for the new matrix.
	 * @param {Number} [ty=0] Specifies the ty property for the new matrix.
	 * @param {Number} [alpha=1] desired alpha value
	 * @param {Shadow} [shadow=null] desired shadow value
	 * @param {String} [compositeOperation=null] desired composite operation value
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	*/
	p.reinitialize = function(a, b, c, d, tx, ty, alpha, shadow, compositeOperation) {
		this.initialize(a,b,c,d,tx,ty);
		this.alpha = alpha == null ? 1 : alpha;
		this.shadow = shadow;
		this.compositeOperation = compositeOperation;
		return this;
	};
	
	/**
	 * Copies all properties from the specified matrix to this matrix.
	 * @method copy
	 * @param {Matrix2D} matrix The matrix to copy properties from.
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	*/
	p.copy = function(matrix) {
		return this.reinitialize(matrix.a, matrix.b, matrix.c, matrix.d, matrix.tx, matrix.ty, matrix.alpha, matrix.shadow, matrix.compositeOperation);
	};

	/**
	 * Appends the specified visual properties to the current matrix.
	 * @method appendProperties
	 * @param {Number} alpha desired alpha value
	 * @param {Shadow} shadow desired shadow value
	 * @param {String} compositeOperation desired composite operation value
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	*/
	p.appendProperties = function(alpha, shadow, compositeOperation) {
		this.alpha *= alpha;
		this.shadow = shadow || this.shadow;
		this.compositeOperation = compositeOperation || this.compositeOperation;
		return this;
	};

	/**
	 * Prepends the specified visual properties to the current matrix.
	 * @method prependProperties
	 * @param {Number} alpha desired alpha value
	 * @param {Shadow} shadow desired shadow value
	 * @param {String} compositeOperation desired composite operation value
	 * @return {Matrix2D} This matrix. Useful for chaining method calls.
	*/
	p.prependProperties = function(alpha, shadow, compositeOperation) {
		this.alpha *= alpha;
		this.shadow = this.shadow || shadow;
		this.compositeOperation = this.compositeOperation || compositeOperation;
		return this;
	};

	/**
	 * Returns a clone of the Matrix2D instance.
	 * @method clone
	 * @return {Matrix2D} a clone of the Matrix2D instance.
	 **/
	p.clone = function() {
		return (new Matrix2D()).copy(this);
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Matrix2D (a="+this.a+" b="+this.b+" c="+this.c+" d="+this.d+" tx="+this.tx+" ty="+this.ty+")]";
	};

	// this has to be populated after the class is defined:
	Matrix2D.identity = new Matrix2D();

createjs.Matrix2D = Matrix2D;
}());
/*
* Point
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Represents a point on a 2 dimensional x / y coordinate system.
 *
 * <h4>Example</h4>
 *      var point = new Point(0, 100);
 *
 * @class Point
 * @param {Number} [x=0] X position.
 * @param {Number} [y=0] Y position.
 * @constructor
 **/
var Point = function(x, y) {
  this.initialize(x, y);
};
var p = Point.prototype;

// public properties:

	/**
	 * X position.
	 * @property x
	 * @type Number
	 **/
	p.x = 0;

	/**
	 * Y position.
	 * @property y
	 * @type Number
	 **/
	p.y = 0;

// constructor:
	/** 
	 * Initialization method. Can also be used to reinitialize the instance.
	 * @method initialize
	 * @param {Number} [x=0] X position.
	 * @param {Number} [y=0] Y position.
	 * @return {Point} This instance. Useful for chaining method calls.
	*/
	p.initialize = function(x, y) {
		this.x = (x == null ? 0 : x);
		this.y = (y == null ? 0 : y);
		return this;
	};
	
// public methods:
	/**
	 * Copies all properties from the specified point to this point.
	 * @method copy
	 * @param {Point} point The point to copy properties from.
	 * @return {Point} This point. Useful for chaining method calls.
	*/
	p.copy = function(point) {
		return this.initialize(point.x, point.y);
	};
	
	/**
	 * Returns a clone of the Point instance.
	 * @method clone
	 * @return {Point} a clone of the Point instance.
	 **/
	p.clone = function() {
		return new Point(this.x, this.y);
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Point (x="+this.x+" y="+this.y+")]";
	};
	
createjs.Point = Point;
}());
/*
* Rectangle
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Represents a rectangle as defined by the points (x, y) and (x+width, y+height).
 *
 * @example
 *      var rect = new createjs.Rectangle(0, 0, 100, 100);
 *
 * @class Rectangle
 * @param {Number} [x=0] X position.
 * @param {Number} [y=0] Y position.
 * @param {Number} [width=0] The width of the Rectangle.
 * @param {Number} [height=0] The height of the Rectangle.
 * @constructor
 **/
var Rectangle = function(x, y, width, height) {
  this.initialize(x, y, width, height);
};
var p = Rectangle.prototype;

// public properties:
	/**
	 * X position.
	 * @property x
	 * @type Number
	 **/
	p.x = 0;

	/**
	 * Y position.
	 * @property y
	 * @type Number
	 **/
	p.y = 0;

	/**
	 * Width.
	 * @property width
	 * @type Number
	 **/
	p.width = 0;

	/**
	 * Height.
	 * @property height
	 * @type Number
	 **/
	p.height = 0;

// constructor:
	/** 
	 * Initialization method. Can also be used to reinitialize the instance.
	 * @method initialize
	 * @param {Number} [x=0] X position.
	 * @param {Number} [y=0] Y position.
	 * @param {Number} [width=0] The width of the Rectangle.
	 * @param {Number} [height=0] The height of the Rectangle.
	 * @return {Rectangle} This instance. Useful for chaining method calls.
	*/
	p.initialize = function(x, y, width, height) {
		this.x = x||0;
		this.y = y||0;
		this.width = width||0;
		this.height = height||0;
		return this;
	};
	
// public methods:
	/**
	 * Copies all properties from the specified rectangle to this rectangle.
	 * @method copy
	 * @param {Rectangle} rectangle The rectangle to copy properties from.
	 * @return {Rectangle} This rectangle. Useful for chaining method calls.
	*/
	p.copy = function(rectangle) {
		return this.initialize(rectangle.x, rectangle.y, rectangle.width, rectangle.height);
	};
	
	/**
	 * Returns a clone of the Rectangle instance.
	 * @method clone
	 * @return {Rectangle} a clone of the Rectangle instance.
	 **/
	p.clone = function() {
		return new Rectangle(this.x, this.y, this.width, this.height);
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Rectangle (x="+this.x+" y="+this.y+" width="+this.width+" height="+this.height+")]";
	};
	
createjs.Rectangle = Rectangle;
}());
/*
* ButtonHelper
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * The ButtonHelper is a helper class to create interactive buttons from {{#crossLink "MovieClip"}}{{/crossLink}} or
 * {{#crossLink "Sprite"}}{{/crossLink}} instances. This class will intercept mouse events from an object, and
 * automatically call {{#crossLink "Sprite/gotoAndStop"}}{{/crossLink}} or {{#crossLink "Sprite/gotoAndPlay"}}{{/crossLink}},
 * to the respective animation labels, add a pointer cursor, and allows the user to define a hit state frame.
 *
 * The ButtonHelper instance does not need to be added to the stage, but a reference should be maintained to prevent
 * garbage collection.
 * 
 * Note that over states will not work unless you call {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}}.
 *
 * <h4>Example</h4>
 *
 *      var helper = new createjs.ButtonHelper(myInstance, "out", "over", "down", false, myInstance, "hit");
 *      myInstance.addEventListener("click", handleClick);
 *      function handleClick(event) {
 *          // Click Happened.
 *      }
 *
 * @class ButtonHelper
 * @param {Sprite|MovieClip} target The instance to manage.
 * @param {String} [outLabel="out"] The label or animation to go to when the user rolls out of the button.
 * @param {String} [overLabel="over"] The label or animation to go to when the user rolls over the button.
 * @param {String} [downLabel="down"] The label or animation to go to when the user presses the button.
 * @param {Boolean} [play=false] If the helper should call "gotoAndPlay" or "gotoAndStop" on the button when changing
 * states.
 * @param {DisplayObject} [hitArea] An optional item to use as the hit state for the button. If this is not defined,
 * then the button's visible states will be used instead. Note that the same instance as the "target" argument can be
 * used for the hitState.
 * @param {String} [hitLabel] The label or animation on the hitArea instance that defines the hitArea bounds. If this is
 * null, then the default state of the hitArea will be used. *
 * @constructor
 */
var ButtonHelper = function(target, outLabel, overLabel, downLabel, play, hitArea, hitLabel) {
	this.initialize(target, outLabel, overLabel, downLabel, play, hitArea, hitLabel);
};
var p = ButtonHelper.prototype;

// public properties:
	/**
	 * The target for this button helper.
	 * @property target
	 * @type MovieClip | Sprite
	 * @readonly
	 **/
	p.target = null;

	/**
	 * The label name or frame number to display when the user mouses out of the target. Defaults to "over".
	 * @property overLabel
	 * @type String | Number
	 **/
	p.overLabel = null;

	/**
	 * The label name or frame number to display when the user mouses over the target. Defaults to "out".
	 * @property outLabel
	 * @type String | Number
	 **/
	p.outLabel = null;

	/**
	 * The label name or frame number to display when the user presses on the target. Defaults to "down".
	 * @property downLabel
	 * @type String | Number
	 **/
	p.downLabel = null;

	/**
	 * If true, then ButtonHelper will call gotoAndPlay, if false, it will use gotoAndStop. Default is false.
	 * @property play
	 * @default false
	 * @type Boolean
	 **/
	p.play = false;

//  private properties
	/**
	 * @property _isPressed
	 * @type Boolean
	 * @protected
	 **/
	p._isPressed = false;

	/**
	 * @property _isOver
	 * @type Boolean
	 * @protected
	 **/
	p._isOver = false;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @param {Sprite|MovieClip} target The instance to manage.
	 * @param {String} [outLabel="out"] The label or animation to go to when the user rolls out of the button.
	 * @param {String} [overLabel="over"] The label or animation to go to when the user rolls over the button.
	 * @param {String} [downLabel="down"] The label or animation to go to when the user presses the button.
	 * @param {Boolean} [play=false] If the helper should call "gotoAndPlay" or "gotoAndStop" on the button when changing
	 * states.
	 * @param {DisplayObject} [hitArea] An optional item to use as the hit state for the button. If this is not defined,
	 * then the button's visible states will be used instead. Note that the same instance as the "target" argument can be
	 * used for the hitState.
	 * @param {String} [hitLabel] The label or animation on the hitArea instance that defines the hitArea bounds. If this is
	 * null, then the default state of the hitArea will be used.
	 * @protected
	 **/
	p.initialize = function(target, outLabel, overLabel, downLabel, play, hitArea, hitLabel) {
		if (!target.addEventListener) { return; }
		this.target = target;
		target.cursor = "pointer";
		this.overLabel = overLabel == null ? "over" : overLabel;
		this.outLabel = outLabel == null ? "out" : outLabel;
		this.downLabel = downLabel == null ? "down" : downLabel;
		this.play = play;
		this.setEnabled(true);
		this.handleEvent({});
		if (hitArea) {
			if (hitLabel) {
				hitArea.actionsEnabled = false;
				hitArea.gotoAndStop&&hitArea.gotoAndStop(hitLabel);
			}
			target.hitArea = hitArea;
		}
	};

// public methods:
	/**
	 * Enables or disables the button functionality on the target.
	 * @method setEnabled
	 * @param {Boolean} value
	 **/
	p.setEnabled = function(value) {
		var o = this.target;
		if (value) {
			o.addEventListener("rollover", this);
			o.addEventListener("rollout", this);
			o.addEventListener("mousedown", this);
			o.addEventListener("pressup", this);
		} else {
			o.removeEventListener("rollover", this);
			o.removeEventListener("rollout", this);
			o.removeEventListener("mousedown", this);
			o.removeEventListener("pressup", this);
		}
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[ButtonHelper]";
	};


// protected methods:
	/**
	 * @method handleEvent
	 * @param {Object} evt The mouse event to handle.
	 * @protected
	 **/
	p.handleEvent = function(evt) {
		var label, t = this.target, type = evt.type;

		if (type == "mousedown") {
			this._isPressed = true;
			label = this.downLabel;
		} else if (type == "pressup") {
			this._isPressed = false;
			label = this._isOver ? this.overLabel : this.outLabel;
		} else if (type == "rollover") {
			this._isOver = true;
			label = this._isPressed ? this.downLabel : this.overLabel;
		} else { // rollout and default
			this._isOver = false;
			label = this._isPressed ? this.overLabel : this.outLabel;
		}
		if (this.play) {
			t.gotoAndPlay&&t.gotoAndPlay(label);
		} else {
			t.gotoAndStop&&t.gotoAndStop(label);
		}
	};

createjs.ButtonHelper = ButtonHelper;
}());
/*
* Shadow
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * This class encapsulates the properties required to define a shadow to apply to a {{#crossLink "DisplayObject"}}{{/crossLink}}
 * via its <code>shadow</code> property.
 *
 * <h4>Example</h4>
 *      myImage.shadow = new createjs.Shadow("#000000", 5, 5, 10);
 *
 * @class Shadow
 * @constructor
 * @param {String} color The color of the shadow.
 * @param {Number} offsetX The x offset of the shadow in pixels.
 * @param {Number} offsetY The y offset of the shadow in pixels.
 * @param {Number} blur The size of the blurring effect.
 **/
var Shadow = function(color, offsetX, offsetY, blur) {
  this.initialize(color, offsetX, offsetY, blur);
};
var p = Shadow.prototype;

// static public properties:
	/**
	 * An identity shadow object (all properties are set to 0).
	 * @property identity
	 * @type Shadow
	 * @static
	 * @final
	 * @readonly
	 **/
	Shadow.identity = null; // set at bottom of class definition.

// public properties:
	/** The color of the shadow.
	 * property color
	 * @type String
	 * @default null
	 */
	p.color = null;

	/** The x offset of the shadow.
	 * property offsetX
	 * @type Number
	 * @default 0
	 */
	p.offsetX = 0;

	/** The y offset of the shadow.
	 * property offsetY
	 * @type Number
	 * @default 0
	 */
	p.offsetY = 0;

	/** The blur of the shadow.
	 * property blur
	 * @type Number
	 * @default 0
	 */
	p.blur = 0;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	 * @param {String} color The color of the shadow.
	 * @param {Number} offsetX The x offset of the shadow.
	 * @param {Number} offsetY The y offset of the shadow.
	 * @param {Number} blur The size of the blurring effect.
	 **/
	p.initialize = function(color, offsetX, offsetY, blur) {
		this.color = color;
		this.offsetX = offsetX;
		this.offsetY = offsetY;
		this.blur = blur;
	};

// public methods:
	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Shadow]";
	};


	/**
	 * Returns a clone of this Shadow instance.
	 * @method clone
	 * @return {Shadow} A clone of the current Shadow instance.
	 **/
	p.clone = function() {
		return new Shadow(this.color, this.offsetX, this.offsetY, this.blur);
	};

	// this has to be populated after the class is defined:
	Shadow.identity = new Shadow("transparent", 0, 0, 0);

createjs.Shadow = Shadow;
}());
/*
* SpriteSheet
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";
/**
 * Encapsulates the properties and methods associated with a sprite sheet. A sprite sheet is a series of images (usually
 * animation frames) combined into a larger image (or images). For example, an animation consisting of eight 100x100
 * images could be combined into a single 400x200 sprite sheet (4 frames across by 2 high).
 *
 * The data passed to the SpriteSheet constructor defines three critical pieces of information:<ol>
 *    <li> The image or images to use.</li>
 *    <li> The positions of individual image frames. This data can be represented in one of two ways:
 *    As a regular grid of sequential, equal-sized frames, or as individually defined, variable sized frames arranged in
 *    an irregular (non-sequential) fashion.</li>
 *    <li> Likewise, animations can be represented in two ways: As a series of sequential frames, defined by a start and
 *    end frame [0,3], or as a list of frames [0,1,2,3].</li>
 * </OL>
 *
 * <h4>SpriteSheet Format</h4>
 *
 *      data = {
 *          // DEFINING FRAMERATE:
 *          // this specifies the framerate that will be set on the SpriteSheet. See {{#crossLink "SpriteSheet/framerate:property"}}{{/crossLink}}
 *          // for more information.
 *          framerate: 20,
 *
 *          // DEFINING IMAGES:
 *          // list of images or image URIs to use. SpriteSheet can handle preloading.
 *          // the order dictates their index value for frame definition.
 *          images: [image1, "path/to/image2.png"],
 *
 *          // DEFINING FRAMES:
 * 	        // the simple way to define frames, only requires frame size because frames are consecutive:
 * 	        // define frame width/height, and optionally the frame count and registration point x/y.
 * 	        // if count is omitted, it will be calculated automatically based on image dimensions.
 * 	        frames: {width:64, height:64, count:20, regX: 32, regY:64},
 *
 * 	        // OR, the complex way that defines individual rects for frames.
 * 	        // The 5th value is the image index per the list defined in "images" (defaults to 0).
 * 	        frames: [
 * 	        	// x, y, width, height, imageIndex, regX, regY
 * 	        	[0,0,64,64,0,32,64],
 * 	        	[64,0,96,64,0]
 * 	        ],
 *
 *          // DEFINING ANIMATIONS:
 *
 * 	        // simple animation definitions. Define a consecutive range of frames (begin to end inclusive).
 * 	        // optionally define a "next" animation to sequence to (or false to stop) and a playback "speed".
 * 	        animations: {
 * 	        	// start, end, next, speed
 * 	        	run: [0,8],
 * 	        	jump: [9,12,"run",2]
 * 	        }
 *
 *          // the complex approach which specifies every frame in the animation by index.
 *          animations: {
 *          	run: {
 *          		frames: [1,2,3,3,2,1]
 *          	},
 *          	jump: {
 *          		frames: [1,4,5,6,1],
 *          		next: "run",
 *          		speed: 2
 *          	},
 *          	stand: { frames: [7] }
 *          }
 *
 * 	        // the above two approaches can be combined, you can also use a single frame definition:
 * 	        animations: {
 * 	        	run: [0,8,true,2],
 * 	        	jump: {
 * 	        		frames: [8,9,10,9,8],
 * 	        		next: "run",
 * 	        		speed: 2
 * 	        	},
 * 	        	stand: 7
 * 	        }
 *      }
 *
 * <strong>Note that the <code>speed</code> property was added in EaselJS 0.7.0. Earlier versions had a <code>frequency</code>
 * property instead, which was the inverse of speed. For example, a value of "4" would be 1/4 normal speed in earlier
 * versions, but us 4x normal speed in 0.7.0+.</strong>
 *
 * <h4>Example</h4>
 * To define a simple sprite sheet, with a single image "sprites.jpg" arranged in a regular 50x50 grid with two
 * animations, "run" looping from frame 0-4 inclusive, and "jump" playing from frame 5-8 and sequencing back to run:
 *
 *      var data = {
 *          images: ["sprites.jpg"],
 *          frames: {width:50, height:50},
 *          animations: {run:[0,4], jump:[5,8,"run"]}
 *      };
 *      var spriteSheet = new createjs.SpriteSheet(data);
 *      var animation = new createjs.Sprite(spriteSheet, "run");
 *
 *
 * <strong>Warning:</strong> Images loaded cross-origin will throw cross-origin security errors when interacted with
 * using a mouse, using methods such as `getObjectUnderPoint`, using filters, or caching. You can get around this by
 * setting `crossOrigin` flags on your images before passing them to EaselJS, eg: `img.crossOrigin="Anonymous";`
 *
 * @class SpriteSheet
 * @constructor
 * @param {Object} data An object describing the SpriteSheet data.
 * @extends EventDispatcher
 **/
var SpriteSheet = function(data) {
  this.initialize(data);
};
var p = SpriteSheet.prototype = new createjs.EventDispatcher();

// events:

	/**
	 * Dispatched when all images are loaded.  Note that this only fires if the images
	 * were not fully loaded when the sprite sheet was initialized. You should check the complete property
	 * to prior to adding a listener. Ex.
	 * <pre><code>var sheet = new SpriteSheet(data);
	 * if (!sheet.complete) {
	 *  &nbsp; // not preloaded, listen for the complete event:
	 *  &nbsp; sheet.addEventListener("complete", handler);
	 * }</code></pre>
	 * @event complete
	 * @param {Object} target The object that dispatched the event.
	 * @param {String} type The event type.
	 * @since 0.6.0
	 */

// public properties:
	/**
	 * Indicates whether all images are finished loading.
	 * @property complete
	 * @type Boolean
	 * @readonly
	 **/
	p.complete = true;


	/**
	 * Specifies the framerate to use by default for Sprite instances using the SpriteSheet. See
	 * Sprite.framerate for more information.
	 * @property framerate
	 * @type Number
	 **/
	p.framerate = 0;

	// TODO: deprecated.
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "SpriteSheet/complete:event"}}{{/crossLink}}
	 * event.
	 * @property onComplete
	 * @type Function
	 * @deprecated Use addEventListener and the "complete" event.
	 **/

// private properties:
	/**
	 * @property _animations
	 * @protected
	 **/
	p._animations = null;

	/**
	 * @property _frames
	 * @protected
	 **/
	p._frames = null;

	/**
	 * @property _images
	 * @protected
	 **/
	p._images = null;

	/**
	 * @property _data
	 * @protected
	 **/
	p._data = null;

	/**
	 * @property _loadCount
	 * @protected
	 **/
	p._loadCount = 0;

	// only used for simple frame defs:
	/**
	 * @property _frameHeight
	 * @protected
	 **/
	p._frameHeight = 0;

	/**
	 * @property _frameWidth
	 * @protected
	 **/
	p._frameWidth = 0;

	/**
	 * @property _numFrames
	 * @protected
	 **/
	p._numFrames = 0;

	/**
	 * @property _regX
	 * @protected
	 **/
	p._regX = 0;

	/**
	 * @property _regY
	 * @protected
	 **/
	p._regY = 0;

// constructor:
	/**
	 * @method initialize
	 * @param {Object} data An object describing the SpriteSheet data.
	 * @protected
	 **/
	p.initialize = function(data) {
		var i,l,o,a;
		if (data == null) { return; }

		this.framerate = data.framerate||0;

		// parse images:
		if (data.images && (l=data.images.length) > 0) {
			a = this._images = [];
			for (i=0; i<l; i++) {
				var img = data.images[i];
				if (typeof img == "string") {
					var src = img;
					img = document.createElement("img");
					img.src = src;
				}
				a.push(img);
				if (!img.getContext && !img.complete) {
					this._loadCount++;
					this.complete = false;
					(function(o) { img.onload = function() { o._handleImageLoad(); } })(this);
				}
			}
		}

		// parse frames:
		if (data.frames == null) { // nothing
		} else if (data.frames instanceof Array) {
			this._frames = [];
			a = data.frames;
			for (i=0,l=a.length;i<l;i++) {
				var arr = a[i];
				this._frames.push({image:this._images[arr[4]?arr[4]:0], rect:new createjs.Rectangle(arr[0],arr[1],arr[2],arr[3]), regX:arr[5]||0, regY:arr[6]||0 });
			}
		} else {
			o = data.frames;
			this._frameWidth = o.width;
			this._frameHeight = o.height;
			this._regX = o.regX||0;
			this._regY = o.regY||0;
			this._numFrames = o.count;
			if (this._loadCount == 0) { this._calculateFrames(); }
		}
		
		// parse animations:
		this._animations = [];
		if ((o=data.animations) != null) {
			this._data = {};
			var name;
			for (name in o) {
				var anim = {name:name};
				var obj = o[name];
				if (typeof obj == "number") { // single frame
					a = anim.frames = [obj];
				} else if (obj instanceof Array) { // simple
					if (obj.length == 1) { anim.frames = [obj[0]]; }
					else {
						anim.speed = obj[3];
						anim.next = obj[2];
						a = anim.frames = [];
						for (i=obj[0];i<=obj[1];i++) {
							a.push(i);
						}
					}
				} else { // complex
					anim.speed = obj.speed;
					anim.next = obj.next;
					var frames = obj.frames;
					a = anim.frames = (typeof frames == "number") ? [frames] : frames.slice(0);
				}
				if (anim.next === true || anim.next === undefined) { anim.next = name; } // loop
				if (anim.next === false || (a.length < 2 && anim.next == name)) { anim.next = null; } // stop
				if (!anim.speed) { anim.speed = 1; }
				this._animations.push(name);
				this._data[name] = anim;
			}
		}

	};

// public methods:
	/**
	 * Returns the total number of frames in the specified animation, or in the whole sprite
	 * sheet if the animation param is omitted.
     * @method getNumFrames
	 * @param {String} animation The name of the animation to get a frame count for.
	 * @return {Number} The number of frames in the animation, or in the entire sprite sheet if the animation param is omitted.
	*/
	p.getNumFrames = function(animation) {
		if (animation == null) {
			return this._frames ? this._frames.length : this._numFrames;
		} else {
			var data = this._data[animation];
			if (data == null) { return 0; }
			else { return data.frames.length; }
		}
	};

	/**
	 * Returns an array of all available animation names as strings.
	 * @method getAnimations
	 * @return {Array} an array of animation names available on this sprite sheet.
	 **/
	p.getAnimations = function() {
		return this._animations.slice(0);
	};

	/**
	 * Returns an object defining the specified animation. The returned object contains:<UL>
	 *     <LI>frames: an array of the frame ids in the animation</LI>
	 *     <LI>speed: the playback speed for this animation</LI>
	 *     <LI>name: the name of the animation</LI>
	 *     <LI>next: the default animation to play next. If the animation loops, the name and next property will be the
	 *     same.</LI>
	 * </UL>
	 * @method getAnimation
	 * @param {String} name The name of the animation to get.
	 * @return {Object} a generic object with frames, speed, name, and next properties.
	 **/
	p.getAnimation = function(name) {
		return this._data[name];
	};

	/**
	 * Returns an object specifying the image and source rect of the specified frame. The returned object has:<UL>
	 *     <LI>an image property holding a reference to the image object in which the frame is found</LI>
	 *     <LI>a rect property containing a Rectangle instance which defines the boundaries for the frame within that
	 *     image.</LI>
	 * </UL>
	 * @method getFrame
	 * @param {Number} frameIndex The index of the frame.
	 * @return {Object} a generic object with image and rect properties. Returns null if the frame does not exist.
	 **/
	p.getFrame = function(frameIndex) {
		var frame;
		if (this._frames && (frame=this._frames[frameIndex])) { return frame; }
		return null;
	};

	/**
	 * Returns a {{#crossLink "Rectangle"}}{{/crossLink}} instance defining the bounds of the specified frame relative
	 * to the origin. For example, a 90 x 70 frame with a regX of 50 and a regY of 40 would return:
	 *
	 *      [x=-50, y=-40, width=90, height=70]
	 *
	 * @method getFrameBounds
	 * @param {Number} frameIndex The index of the frame.
	 * @param {Rectangle} [rectangle] A Rectangle instance to copy the values into. By default a new instance is created.
	 * @return {Rectangle} A Rectangle instance. Returns null if the frame does not exist, or the image is not fully loaded.
	 **/
	p.getFrameBounds = function(frameIndex, rectangle) {
		var frame = this.getFrame(frameIndex);
		return frame ? (rectangle||new createjs.Rectangle()).initialize(-frame.regX, -frame.regY, frame.rect.width, frame.rect.height) : null;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[SpriteSheet]";
	};

	/**
	 * Returns a clone of the SpriteSheet instance.
	 * @method clone
	 * @return {SpriteSheet} a clone of the SpriteSheet instance.
	 **/
	p.clone = function() {
		// TODO: there isn't really any reason to clone SpriteSheet instances, because they can be reused.
		var o = new SpriteSheet();
		o.complete = this.complete;
		o._animations = this._animations;
		o._frames = this._frames;
		o._images = this._images;
		o._data = this._data;
		o._frameHeight = this._frameHeight;
		o._frameWidth = this._frameWidth;
		o._numFrames = this._numFrames;
		o._loadCount = this._loadCount;
		return o;
	};

// private methods:
	/**
	 * @method _handleImageLoad
	 * @protected
	 **/
	p._handleImageLoad = function() {
		if (--this._loadCount == 0) {
			this._calculateFrames();
			this.complete = true;
			this.dispatchEvent("complete");
		}
	};

	/**
	 * @method _calculateFrames
	 * @protected
	 **/
	p._calculateFrames = function() {
		if (this._frames || this._frameWidth == 0) { return; }
		this._frames = [];
		var ttlFrames = 0;
		var fw = this._frameWidth;
		var fh = this._frameHeight;
		for (var i=0,imgs = this._images; i<imgs.length; i++) {
			var img = imgs[i];
			var cols = img.width/fw|0;
			var rows = img.height/fh|0;
			var ttl = this._numFrames>0 ? Math.min(this._numFrames-ttlFrames,cols*rows) : cols*rows;
			for (var j=0;j<ttl;j++) {
				this._frames.push({image:img, rect:new createjs.Rectangle(j%cols*fw,(j/cols|0)*fh,fw,fh), regX:this._regX, regY:this._regY });
			}
			ttlFrames += ttl;
		}
		this._numFrames = ttlFrames;
	};

createjs.SpriteSheet = SpriteSheet;
}());
/*
* Graphics
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
* Inner class used by the {{#crossLink "Graphics"}}{{/crossLink}} class. Used to create the instruction lists used in Graphics:
* @class Command
* @protected
* @constructor
**/
function Command(f, params, path) {
	this.f = f;
	this.params = params;
	this.path = path==null ? true : path;
}

/**
* @method exec
* @protected
* @param {Object} scope
**/
Command.prototype.exec = function(scope) { this.f.apply(scope, this.params); };

/**
 * The Graphics class exposes an easy to use API for generating vector drawing instructions and drawing them to a
 * specified context. Note that you can use Graphics without any dependency on the Easel framework by calling {{#crossLink "DisplayObject/draw"}}{{/crossLink}}
 * directly, or it can be used with the {{#crossLink "Shape"}}{{/crossLink}} object to draw vector graphics within the
 * context of an Easel display list.
 *
 * <h4>Example</h4>
 *      var g = new createjs.Graphics();
 *	    g.setStrokeStyle(1);
 *	    g.beginStroke(createjs.Graphics.getRGB(0,0,0));
 *	    g.beginFill(createjs.Graphics.getRGB(255,0,0));
 *	    g.drawCircle(0,0,3);
 *
 *	    var s = new createjs.Shape(g);
 *	    	s.x = 100;
 *	    	s.y = 100;
 *
 *	    stage.addChild(s);
 *	    stage.update();
 *
 * Note that all drawing methods in Graphics return the Graphics instance, so they can be chained together. For example,
 * the following line of code would generate the instructions to draw a rectangle with a red stroke and blue fill, then
 * render it to the specified context2D:
 *
 *      myGraphics.beginStroke("#F00").beginFill("#00F").drawRect(20, 20, 100, 50).draw(myContext2D);
 *
 * <h4>Tiny API</h4>
 * The Graphics class also includes a "tiny API", which is one or two-letter methods that are shortcuts for all of the
 * Graphics methods. These methods are great for creating compact instructions, and is used by the Toolkit for CreateJS
 * to generate readable code. All tiny methods are marked as protected, so you can view them by enabling protected
 * descriptions in the docs.
 *
 * <table>
 *     <tr><td><b>Tiny</b></td><td><b>Method</b></td><td><b>Tiny</b></td><td><b>Method</b></td></tr>
 *     <tr><td>mt</td><td>{{#crossLink "Graphics/moveTo"}}{{/crossLink}} </td>
 *     <td>lt</td> <td>{{#crossLink "Graphics/lineTo"}}{{/crossLink}}</td></tr>
 *     <tr><td>a/at</td><td>{{#crossLink "Graphics/arc"}}{{/crossLink}} / {{#crossLink "Graphics/arcTo"}}{{/crossLink}} </td>
 *     <td>bt</td><td>{{#crossLink "Graphics/bezierCurveTo"}}{{/crossLink}} </td></tr>
 *     <tr><td>qt</td><td>{{#crossLink "Graphics/quadraticCurveTo"}}{{/crossLink}} (also curveTo)</td>
 *     <td>r</td><td>{{#crossLink "Graphics/rect"}}{{/crossLink}} </td></tr>
 *     <tr><td>cp</td><td>{{#crossLink "Graphics/closePath"}}{{/crossLink}} </td>
 *     <td>c</td><td>{{#crossLink "Graphics/clear"}}{{/crossLink}} </td></tr>
 *     <tr><td>f</td><td>{{#crossLink "Graphics/beginFill"}}{{/crossLink}} </td>
 *     <td>lf</td><td>{{#crossLink "Graphics/beginLinearGradientFill"}}{{/crossLink}} </td></tr>
 *     <tr><td>rf</td><td>{{#crossLink "Graphics/beginRadialGradientFill"}}{{/crossLink}} </td>
 *     <td>bf</td><td>{{#crossLink "Graphics/beginBitmapFill"}}{{/crossLink}} </td></tr>
 *     <tr><td>ef</td><td>{{#crossLink "Graphics/endFill"}}{{/crossLink}} </td>
 *     <td>ss</td><td>{{#crossLink "Graphics/setStrokeStyle"}}{{/crossLink}} </td></tr>
 *     <tr><td>s</td><td>{{#crossLink "Graphics/beginStroke"}}{{/crossLink}} </td>
 *     <td>ls</td><td>{{#crossLink "Graphics/beginLinearGradientStroke"}}{{/crossLink}} </td></tr>
 *     <tr><td>rs</td><td>{{#crossLink "Graphics/beginRadialGradientStroke"}}{{/crossLink}} </td>
 *     <td>bs</td><td>{{#crossLink "Graphics/beginBitmapStroke"}}{{/crossLink}} </td></tr>
 *     <tr><td>es</td><td>{{#crossLink "Graphics/endStroke"}}{{/crossLink}} </td>
 *     <td>dr</td><td>{{#crossLink "Graphics/drawRect"}}{{/crossLink}} </td></tr>
 *     <tr><td>rr</td><td>{{#crossLink "Graphics/drawRoundRect"}}{{/crossLink}} </td>
 *     <td>rc</td><td>{{#crossLink "Graphics/drawRoundRectComplex"}}{{/crossLink}} </td></tr>
 *     <tr><td>dc</td><td>{{#crossLink "Graphics/drawCircle"}}{{/crossLink}} </td>
 *     <td>de</td><td>{{#crossLink "Graphics/drawEllipse"}}{{/crossLink}} </td></tr>
 *     <tr><td>dp</td><td>{{#crossLink "Graphics/drawPolyStar"}}{{/crossLink}} </td>
 *     <td>p</td><td>{{#crossLink "Graphics/decodePath"}}{{/crossLink}} </td></tr>
 * </table>
 *
 * Here is the above example, using the tiny API instead.
 *
 *      myGraphics.s("#F00").f("#00F").r(20, 20, 100, 50).draw(myContext2D);
 *
 * @class Graphics
 * @constructor
 * @for Graphics
 **/
var Graphics = function() {
	this.initialize();
};
var p = Graphics.prototype;

// static public methods:


	/**
	 * Returns a CSS compatible color string based on the specified RGB numeric color values in the format
	 * "rgba(255,255,255,1.0)", or if alpha is null then in the format "rgb(255,255,255)". For example,
	 *
	 *      createjs.Graphics.getRGB(50, 100, 150, 0.5);
	 *      // Returns "rgba(50,100,150,0.5)"
	 *
	 * It also supports passing a single hex color value as the first param, and an optional alpha value as the second
	 * param. For example,
	 *
	 *      createjs.Graphics.getRGB(0xFF00FF, 0.2);
	 *      // Returns "rgba(255,0,255,0.2)"
	 *
	 * @method getRGB
	 * @static
	 * @param {Number} r The red component for the color, between 0 and 0xFF (255).
	 * @param {Number} g The green component for the color, between 0 and 0xFF (255).
	 * @param {Number} b The blue component for the color, between 0 and 0xFF (255).
	 * @param {Number} [alpha] The alpha component for the color where 0 is fully transparent and 1 is fully opaque.
	 * @return {String} A CSS compatible color string based on the specified RGB numeric color values in the format
	 * "rgba(255,255,255,1.0)", or if alpha is null then in the format "rgb(255,255,255)".
	 **/
	Graphics.getRGB = function(r, g, b, alpha) {
		if (r != null && b == null) {
			alpha = g;
			b = r&0xFF;
			g = r>>8&0xFF;
			r = r>>16&0xFF;
		}
		if (alpha == null) {
			return "rgb("+r+","+g+","+b+")";
		} else {
			return "rgba("+r+","+g+","+b+","+alpha+")";
		}
	};

	/**
	 * Returns a CSS compatible color string based on the specified HSL numeric color values in the format "hsla(360,100,100,1.0)",
	 * or if alpha is null then in the format "hsl(360,100,100)".
	 *
	 *      createjs.Graphics.getHSL(150, 100, 70);
	 *      // Returns "hsl(150,100,70)"
	 *
	 * @method getHSL
	 * @static
	 * @param {Number} hue The hue component for the color, between 0 and 360.
	 * @param {Number} saturation The saturation component for the color, between 0 and 100.
	 * @param {Number} lightness The lightness component for the color, between 0 and 100.
	 * @param {Number} [alpha] The alpha component for the color where 0 is fully transparent and 1 is fully opaque.
	 * @return {String} A CSS compatible color string based on the specified HSL numeric color values in the format
	 * "hsla(360,100,100,1.0)", or if alpha is null then in the format "hsl(360,100,100)".
	 **/
	Graphics.getHSL = function(hue, saturation, lightness, alpha) {
		if (alpha == null) {
			return "hsl("+(hue%360)+","+saturation+"%,"+lightness+"%)";
		} else {
			return "hsla("+(hue%360)+","+saturation+"%,"+lightness+"%,"+alpha+")";
		}
	};

// static properties:

	/**
	 * Exposes the Command class used internally by Graphics. Useful for extending the Graphics class or injecting
	 * functionality.
	 * @property Command
	 * @static
	 * @type {Function}
	 **/
	Graphics.Command = Command;

	/**
	 * Map of Base64 characters to values. Used by {{#crossLink "Graphics/decodePath"}}{{/crossLink}}.
	 * @property BASE_64
	 * @static
	 * @final
	 * @readonly
	 * @type {Object}
	 **/
	Graphics.BASE_64 = {"A":0,"B":1,"C":2,"D":3,"E":4,"F":5,"G":6,"H":7,"I":8,"J":9,"K":10,"L":11,"M":12,"N":13,"O":14,"P":15,"Q":16,"R":17,"S":18,"T":19,"U":20,"V":21,"W":22,"X":23,"Y":24,"Z":25,"a":26,"b":27,"c":28,"d":29,"e":30,"f":31,"g":32,"h":33,"i":34,"j":35,"k":36,"l":37,"m":38,"n":39,"o":40,"p":41,"q":42,"r":43,"s":44,"t":45,"u":46,"v":47,"w":48,"x":49,"y":50,"z":51,"0":52,"1":53,"2":54,"3":55,"4":56,"5":57,"6":58,"7":59,"8":60,"9":61,"+":62,"/":63};


	/**
	 * Maps numeric values for the caps parameter of {{#crossLink "Graphics/setStrokeStyle"}}{{/crossLink}} to
	 * corresponding string values. This is primarily for use with the tiny API. The mappings are as follows: 0 to
	 * "butt", 1 to "round", and 2 to "square".
	 * For example, to set the line caps to "square":
	 *
	 *      myGraphics.ss(16, 2);
	 *
	 * @property STROKE_CAPS_MAP
	 * @static
	 * @final
	 * @readonly
	 * @type {Array}
	 **/
	Graphics.STROKE_CAPS_MAP = ["butt", "round", "square"];

	/**
	 * Maps numeric values for the joints parameter of {{#crossLink "Graphics/setStrokeStyle"}}{{/crossLink}} to
	 * corresponding string values. This is primarily for use with the tiny API. The mappings are as follows: 0 to
	 * "miter", 1 to "round", and 2 to "bevel".
	 * For example, to set the line joints to "bevel":
	 *
	 *      myGraphics.ss(16, 0, 2);
	 *
	 * @property STROKE_JOINTS_MAP
	 * @static
	 * @final
	 * @readonly
	 * @type {Array}
	 **/
	Graphics.STROKE_JOINTS_MAP = ["miter", "round", "bevel"];

	/**
	 * @property _ctx
	 * @static
	 * @protected
	 * @type {CanvasRenderingContext2D}
	 **/
	 
	/**
	 * @property beginCmd
	 * @static
	 * @protected
	 * @type {Command}
	 **/
	 
	/**
	 * @property fillCmd
	 * @static
	 * @protected
	 * @type {Command}
	 **/
	 
	/**
	 * @property strokeCmd
	 * @static
	 * @protected
	 * @type {Command}
	 **/
	var canvas = (createjs.createCanvas?createjs.createCanvas():document.createElement("canvas"));
	if (canvas.getContext) {
		var ctx = Graphics._ctx = canvas.getContext("2d");
		Graphics.beginCmd = new Command(ctx.beginPath, [], false);
		Graphics.fillCmd = new Command(ctx.fill, [], false);
		Graphics.strokeCmd = new Command(ctx.stroke, [], false);
		canvas.width = canvas.height = 1;
	}
	
// public properties

// private properties
	/**
	 * @property _strokeInstructions
	 * @protected
	 * @type {Array}
	 **/
	p._strokeInstructions = null;

	/**
	 * @property _strokeStyleInstructions
	 * @protected
	 * @type {Array}
	 **/
	p._strokeStyleInstructions = null;

	/**
	 * @property _strokeIgnoreScale
	 * @protected
	 * @type Boolean
	 **/
	p._strokeIgnoreScale = false;

	/**
	 * @property _fillInstructions
	 * @protected
	 * @type {Array}
	 **/
	p._fillInstructions = null;

	/**
	 * @property _strokeMatrix
	 * @protected
	 * @type {Array}
	 **/
	p._fillMatrix = null;

	/**
	 * @property _instructions
	 * @protected
	 * @type {Array}
	 **/
	p._instructions = null;

	/**
	 * @property _oldInstructions
	 * @protected
	 * @type {Array}
	 **/
	p._oldInstructions = null;

	/**
	 * @property _activeInstructions
	 * @protected
	 * @type {Array}
	 **/
	p._activeInstructions = null;

	/**
	 * @property _active
	 * @protected
	 * @type {Boolean}
	 * @default false
	 **/
	p._active = false;

	/**
	 * @property _dirty
	 * @protected
	 * @type {Boolean}
	 * @default false
	 **/
	p._dirty = false;

	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	 **/
	p.initialize = function() {
		this.clear();
		this._ctx = Graphics._ctx;
	};

	/**
	 * Returns true if this Graphics instance has no drawing commands.
	 * @method isEmpty
	 * @return {Boolean} Returns true if this Graphics instance has no drawing commands.
	 **/
	p.isEmpty = function() {
		return !(this._instructions.length || this._oldInstructions.length || this._activeInstructions.length);
	};

	/**
	 * Draws the display object into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 **/
	p.draw = function(ctx) {
		if (this._dirty) { this._updateInstructions(); }
		var instr = this._instructions;
		for (var i=0, l=instr.length; i<l; i++) {
			instr[i].exec(ctx);
		}
	};

	/**
	 * Draws only the path described for this Graphics instance, skipping any non-path instructions, including fill and
	 * stroke descriptions. Used by <code>DisplayObject.clippingPath</code> to draw the clipping path, for example.
	 * @method drawAsPath
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 **/
	p.drawAsPath = function(ctx) {
		if (this._dirty) { this._updateInstructions(); }
		var instr, instrs = this._instructions;
		for (var i=0, l=instrs.length; i<l; i++) {
			// the first command is always a beginPath command.
			if ((instr = instrs[i]).path || i==0) { instr.exec(ctx); }
		}
	};

// public methods that map directly to context 2D calls:
	/**
	 * Moves the drawing point to the specified position. A tiny API method "mt" also exists.
	 * @method moveTo
	 * @param {Number} x The x coordinate the drawing point should move to.
	 * @param {Number} y The y coordinate the drawing point should move to.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls).
	 **/
	p.moveTo = function(x, y) {
		this._activeInstructions.push(new Command(this._ctx.moveTo, [x, y]));
		return this;
	};

	/**
	 * Draws a line from the current drawing point to the specified position, which become the new current drawing
	 * point. A tiny API method "lt" also exists.
	 *
	 * For detailed information, read the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#complex-shapes-(paths)">
	 * whatwg spec</a>.
	 * @method lineTo
	 * @param {Number} x The x coordinate the drawing point should draw to.
	 * @param {Number} y The y coordinate the drawing point should draw to.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.lineTo = function(x, y) {
		this._dirty = this._active = true;
		this._activeInstructions.push(new Command(this._ctx.lineTo, [x, y]));
		return this;
	};

	/**
	 * Draws an arc with the specified control points and radius.  For detailed information, read the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#dom-context-2d-arcto">
	 * whatwg spec</a>. A tiny API method "at" also exists.
	 * @method arcTo
	 * @param {Number} x1
	 * @param {Number} y1
	 * @param {Number} x2
	 * @param {Number} y2
	 * @param {Number} radius
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.arcTo = function(x1, y1, x2, y2, radius) {
		this._dirty = this._active = true;
		this._activeInstructions.push(new Command(this._ctx.arcTo, [x1, y1, x2, y2, radius]));
		return this;
	};

	/**
	 * Draws an arc defined by the radius, startAngle and endAngle arguments, centered at the position (x, y). For
	 * example, to draw a full circle with a radius of 20 centered at (100, 100):
	 *
	 *      arc(100, 100, 20, 0, Math.PI*2);
	 *
	 * For detailed information, read the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#dom-context-2d-arc">whatwg spec</a>.
	 * A tiny API method "a" also exists.
	 * @method arc
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Number} radius
	 * @param {Number} startAngle Measured in radians.
	 * @param {Number} endAngle Measured in radians.
	 * @param {Boolean} anticlockwise
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.arc = function(x, y, radius, startAngle, endAngle, anticlockwise) {
		this._dirty = this._active = true;
		if (anticlockwise == null) { anticlockwise = false; }
		this._activeInstructions.push(new Command(this._ctx.arc, [x, y, radius, startAngle, endAngle, anticlockwise]));
		return this;
	};

	/**
	 * Draws a quadratic curve from the current drawing point to (x, y) using the control point (cpx, cpy). For detailed
	 * information, read the <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#dom-context-2d-quadraticcurveto">
	 * whatwg spec</a>. A tiny API method "qt" also exists.
	 * @method quadraticCurveTo
	 * @param {Number} cpx
	 * @param {Number} cpy
	 * @param {Number} x
	 * @param {Number} y
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.quadraticCurveTo = function(cpx, cpy, x, y) {
		this._dirty = this._active = true;
		this._activeInstructions.push(new Command(this._ctx.quadraticCurveTo, [cpx, cpy, x, y]));
		return this;
	};

	/**
	 * Draws a bezier curve from the current drawing point to (x, y) using the control points (cp1x, cp1y) and (cp2x,
	 * cp2y). For detailed information, read the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#dom-context-2d-beziercurveto">
	 * whatwg spec</a>. A tiny API method "bt" also exists.
	 * @method bezierCurveTo
	 * @param {Number} cp1x
	 * @param {Number} cp1y
	 * @param {Number} cp2x
	 * @param {Number} cp2y
	 * @param {Number} x
	 * @param {Number} y
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.bezierCurveTo = function(cp1x, cp1y, cp2x, cp2y, x, y) {
		this._dirty = this._active = true;
		this._activeInstructions.push(new Command(this._ctx.bezierCurveTo, [cp1x, cp1y, cp2x, cp2y, x, y]));
		return this;
	};

	/**
	 * Draws a rectangle at (x, y) with the specified width and height using the current fill and/or stroke.
	 * For detailed information, read the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#dom-context-2d-rect">
	 * whatwg spec</a>. A tiny API method "r" also exists.
	 * @method rect
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Number} w Width of the rectangle
	 * @param {Number} h Height of the rectangle
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.rect = function(x, y, w, h) {
		this._dirty = this._active = true;
		this._activeInstructions.push(new Command(this._ctx.rect, [x, y, w, h]));
		return this;
	};

	/**
	 * Closes the current path, effectively drawing a line from the current drawing point to the first drawing point specified
	 * since the fill or stroke was last set. A tiny API method "cp" also exists.
	 * @method closePath
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.closePath = function() {
		if (this._active) {
			this._dirty = true;
			this._activeInstructions.push(new Command(this._ctx.closePath, []));
		}
		return this;
	};


// public methods that roughly map to Flash graphics APIs:
	/**
	 * Clears all drawing instructions, effectively resetting this Graphics instance. Any line and fill styles will need
	 * to be redefined to draw shapes following a clear call. A tiny API method "c" also exists.
	 * @method clear
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.clear = function() {
		this._instructions = [];
		this._oldInstructions = [];
		this._activeInstructions = [];
		this._strokeStyleInstructions = this._strokeInstructions = this._fillInstructions = this._fillMatrix = null;
		this._active = this._dirty = this._strokeIgnoreScale = false;
		return this;
	};

	/**
	 * Begins a fill with the specified color. This ends the current sub-path. A tiny API method "f" also exists.
	 * @method beginFill
	 * @param {String} color A CSS compatible color value (ex. "red", "#FF0000", or "rgba(255,0,0,0.5)"). Setting to
	 * null will result in no fill.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginFill = function(color) {
		if (this._active) { this._newPath(); }
		this._fillInstructions = color ? [new Command(this._setProp, ["fillStyle", color], false)] : null;
		this._fillMatrix = null;
		return this;
	};

	/**
	 * Begins a linear gradient fill defined by the line (x0, y0) to (x1, y1). This ends the current sub-path. For
	 * example, the following code defines a black to white vertical gradient ranging from 20px to 120px, and draws a
	 * square to display it:
	 *
	 *      myGraphics.beginLinearGradientFill(["#000","#FFF"], [0, 1], 0, 20, 0, 120).drawRect(20, 20, 120, 120);
	 *
	 * A tiny API method "lf" also exists.
	 * @method beginLinearGradientFill
	 * @param {Array} colors An array of CSS compatible color values. For example, ["#F00","#00F"] would define a gradient
	 * drawing from red to blue.
	 * @param {Array} ratios An array of gradient positions which correspond to the colors. For example, [0.1, 0.9] would draw
	 * the first color to 10% then interpolating to the second color at 90%.
	 * @param {Number} x0 The position of the first point defining the line that defines the gradient direction and size.
	 * @param {Number} y0 The position of the first point defining the line that defines the gradient direction and size.
	 * @param {Number} x1 The position of the second point defining the line that defines the gradient direction and size.
	 * @param {Number} y1 The position of the second point defining the line that defines the gradient direction and size.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginLinearGradientFill = function(colors, ratios, x0, y0, x1, y1) {
		if (this._active) { this._newPath(); }
		var o = this._ctx.createLinearGradient(x0, y0, x1, y1);
		for (var i=0, l=colors.length; i<l; i++) {
			o.addColorStop(ratios[i], colors[i]);
		}
		this._fillInstructions = [new Command(this._setProp, ["fillStyle", o], false)];
		this._fillMatrix = null;
		return this;
	};

	/**
	 * Begins a radial gradient fill. This ends the current sub-path. For example, the following code defines a red to
	 * blue radial gradient centered at (100, 100), with a radius of 50, and draws a circle to display it:
	 *
	 *      myGraphics.beginRadialGradientFill(["#F00","#00F"], [0, 1], 100, 100, 0, 100, 100, 50).drawCircle(100, 100, 50);
	 *
	 * A tiny API method "rf" also exists.
	 * @method beginRadialGradientFill
	 * @param {Array} colors An array of CSS compatible color values. For example, ["#F00","#00F"] would define
	 * a gradient drawing from red to blue.
	 * @param {Array} ratios An array of gradient positions which correspond to the colors. For example, [0.1,
	 * 0.9] would draw the first color to 10% then interpolating to the second color at 90%.
	 * @param {Number} x0 Center position of the inner circle that defines the gradient.
	 * @param {Number} y0 Center position of the inner circle that defines the gradient.
	 * @param {Number} r0 Radius of the inner circle that defines the gradient.
	 * @param {Number} x1 Center position of the outer circle that defines the gradient.
	 * @param {Number} y1 Center position of the outer circle that defines the gradient.
	 * @param {Number} r1 Radius of the outer circle that defines the gradient.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginRadialGradientFill = function(colors, ratios, x0, y0, r0, x1, y1, r1) {
		if (this._active) { this._newPath(); }
		var o = this._ctx.createRadialGradient(x0, y0, r0, x1, y1, r1);
		for (var i=0, l=colors.length; i<l; i++) {
			o.addColorStop(ratios[i], colors[i]);
		}
		this._fillInstructions = [new Command(this._setProp, ["fillStyle", o], false)];
		this._fillMatrix = null;
		return this;
	};

	/**
	 * Begins a pattern fill using the specified image. This ends the current sub-path. A tiny API method "bf" also
	 * exists.
	 * @method beginBitmapFill
	 * @param {HTMLImageElement | HTMLCanvasElement | HTMLVideoElement} image The Image, Canvas, or Video object to use
	 * as the pattern.
	 * @param {String} repetition Optional. Indicates whether to repeat the image in the fill area. One of "repeat",
	 * "repeat-x", "repeat-y", or "no-repeat". Defaults to "repeat". Note that Firefox does not support "repeat-x" or
	 * "repeat-y" (latest tests were in FF 20.0), and will default to "repeat".
	 * @param {Matrix2D} matrix Optional. Specifies a transformation matrix for the bitmap fill. This transformation
	 * will be applied relative to the parent transform.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginBitmapFill = function(image, repetition, matrix) {
		if (this._active) { this._newPath(); }
		repetition = repetition || "";
		var o = this._ctx.createPattern(image, repetition);
		this._fillInstructions = [new Command(this._setProp, ["fillStyle", o], false)];
		this._fillMatrix = matrix ? [matrix.a, matrix.b, matrix.c, matrix.d, matrix.tx, matrix.ty] : null;
		return this;
	};

	/**
	 * Ends the current sub-path, and begins a new one with no fill. Functionally identical to <code>beginFill(null)</code>.
	 * A tiny API method "ef" also exists.
	 * @method endFill
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.endFill = function() {
		return this.beginFill();
	};

	/**
	 * Sets the stroke style for the current sub-path. Like all drawing methods, this can be chained, so you can define
	 * the stroke style and color in a single line of code like so:
	 *
	 *      myGraphics.setStrokeStyle(8,"round").beginStroke("#F00");
	 *
	 * A tiny API method "ss" also exists.
	 * @method setStrokeStyle
	 * @param {Number} thickness The width of the stroke.
	 * @param {String | Number} [caps=0] Indicates the type of caps to use at the end of lines. One of butt,
	 * round, or square. Defaults to "butt". Also accepts the values 0 (butt), 1 (round), and 2 (square) for use with
	 * the tiny API.
	 * @param {String | Number} [joints=0] Specifies the type of joints that should be used where two lines meet.
	 * One of bevel, round, or miter. Defaults to "miter". Also accepts the values 0 (miter), 1 (round), and 2 (bevel)
	 * for use with the tiny API.
	 * @param {Number} [miterLimit=10] If joints is set to "miter", then you can specify a miter limit ratio which
	 * controls at what point a mitered joint will be clipped.
	 * @param {Boolean} [ignoreScale=false] If true, the stroke will be drawn at the specified thickness regardless
	 * of active transformations.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.setStrokeStyle = function(thickness, caps, joints, miterLimit, ignoreScale) {
		if (this._active) { this._newPath(); }
		this._strokeStyleInstructions = [
			new Command(this._setProp, ["lineWidth", (thickness == null ? "1" : thickness)], false),
			new Command(this._setProp, ["lineCap", (caps == null ? "butt" : (isNaN(caps) ? caps : Graphics.STROKE_CAPS_MAP[caps]))], false),
			new Command(this._setProp, ["lineJoin", (joints == null ? "miter" : (isNaN(joints) ? joints : Graphics.STROKE_JOINTS_MAP[joints]))], false),
			new Command(this._setProp, ["miterLimit", (miterLimit == null ? "10" : miterLimit)], false)
			];
		this._strokeIgnoreScale = ignoreScale;
		return this;
	};

	/**
	 * Begins a stroke with the specified color. This ends the current sub-path. A tiny API method "s" also exists.
	 * @method beginStroke
	 * @param {String} color A CSS compatible color value (ex. "#FF0000", "red", or "rgba(255,0,0,0.5)"). Setting to
	 * null will result in no stroke.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginStroke = function(color) {
		if (this._active) { this._newPath(); }
		this._strokeInstructions = color ? [new Command(this._setProp, ["strokeStyle", color], false)] : null;
		return this;
	};

	/**
	 * Begins a linear gradient stroke defined by the line (x0, y0) to (x1, y1). This ends the current sub-path. For
	 * example, the following code defines a black to white vertical gradient ranging from 20px to 120px, and draws a
	 * square to display it:
	 *
	 *      myGraphics.setStrokeStyle(10).
	 *          beginLinearGradientStroke(["#000","#FFF"], [0, 1], 0, 20, 0, 120).drawRect(20, 20, 120, 120);
	 *
	 * A tiny API method "ls" also exists.
	 * @method beginLinearGradientStroke
	 * @param {Array} colors An array of CSS compatible color values. For example, ["#F00","#00F"] would define
	 * a gradient drawing from red to blue.
	 * @param {Array} ratios An array of gradient positions which correspond to the colors. For example, [0.1,
	 * 0.9] would draw the first color to 10% then interpolating to the second color at 90%.
	 * @param {Number} x0 The position of the first point defining the line that defines the gradient direction and size.
	 * @param {Number} y0 The position of the first point defining the line that defines the gradient direction and size.
	 * @param {Number} x1 The position of the second point defining the line that defines the gradient direction and size.
	 * @param {Number} y1 The position of the second point defining the line that defines the gradient direction and size.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginLinearGradientStroke = function(colors, ratios, x0, y0, x1, y1) {
		if (this._active) { this._newPath(); }
		var o = this._ctx.createLinearGradient(x0, y0, x1, y1);
		for (var i=0, l=colors.length; i<l; i++) {
			o.addColorStop(ratios[i], colors[i]);
		}
		this._strokeInstructions = [new Command(this._setProp, ["strokeStyle", o], false)];
		return this;
	};


	/**
	 * Begins a radial gradient stroke. This ends the current sub-path. For example, the following code defines a red to
	 * blue radial gradient centered at (100, 100), with a radius of 50, and draws a rectangle to display it:
	 *
	 *      myGraphics.setStrokeStyle(10)
	 *          .beginRadialGradientStroke(["#F00","#00F"], [0, 1], 100, 100, 0, 100, 100, 50)
	 *          .drawRect(50, 90, 150, 110);
	 *
	 * A tiny API method "rs" also exists.
	 * @method beginRadialGradientStroke
	 * @param {Array} colors An array of CSS compatible color values. For example, ["#F00","#00F"] would define
	 * a gradient drawing from red to blue.
	 * @param {Array} ratios An array of gradient positions which correspond to the colors. For example, [0.1,
	 * 0.9] would draw the first color to 10% then interpolating to the second color at 90%, then draw the second color
	 * to 100%.
	 * @param {Number} x0 Center position of the inner circle that defines the gradient.
	 * @param {Number} y0 Center position of the inner circle that defines the gradient.
	 * @param {Number} r0 Radius of the inner circle that defines the gradient.
	 * @param {Number} x1 Center position of the outer circle that defines the gradient.
	 * @param {Number} y1 Center position of the outer circle that defines the gradient.
	 * @param {Number} r1 Radius of the outer circle that defines the gradient.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginRadialGradientStroke = function(colors, ratios, x0, y0, r0, x1, y1, r1) {
		if (this._active) { this._newPath(); }
		var o = this._ctx.createRadialGradient(x0, y0, r0, x1, y1, r1);
		for (var i=0, l=colors.length; i<l; i++) {
			o.addColorStop(ratios[i], colors[i]);
		}
		this._strokeInstructions = [new Command(this._setProp, ["strokeStyle", o], false)];
		return this;
	};

	/**
	 * Begins a pattern fill using the specified image. This ends the current sub-path. Note that unlike bitmap fills,
	 * strokes do not currently support a matrix parameter due to limitations in the canvas API. A tiny API method "bs"
	 * also exists.
	 * @method beginBitmapStroke
	 * @param {HTMLImageElement | HTMLCanvasElement | HTMLVideoElement} image The Image, Canvas, or Video object to use
	 * as the pattern.
	 * @param {String} [repetition=repeat] Optional. Indicates whether to repeat the image in the fill area. One of
	 * "repeat", "repeat-x", "repeat-y", or "no-repeat". Defaults to "repeat".
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.beginBitmapStroke = function(image, repetition) {
		// NOTE: matrix is not supported for stroke because transforms on strokes also affect the drawn stroke width.
		if (this._active) { this._newPath(); }
		repetition = repetition || "";
		var o = this._ctx.createPattern(image, repetition);
		this._strokeInstructions = [new Command(this._setProp, ["strokeStyle", o], false)];
		return this;
	};

	/**
	 * Ends the current sub-path, and begins a new one with no stroke. Functionally identical to <code>beginStroke(null)</code>.
	 * A tiny API method "es" also exists.
	 * @method endStroke
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.endStroke = function() {
		this.beginStroke();
		return this;
	};

	/**
	 * Maps the familiar ActionScript <code>curveTo()</code> method to the functionally similar {{#crossLink "Graphics/quadraticCurveTo"}}{{/crossLink}}
	 * method.
	 * @method curveTo
	 * @type {Function}
	 **/
	p.curveTo = p.quadraticCurveTo;

	/**
	 * Maps the familiar ActionScript <code>drawRect()</code> method to the functionally similar {{#crossLink "Graphics/rect"}}{{/crossLink}}
	 * method.
	 * @method drawRect
	 * @type {Function}
	 **/
	p.drawRect = p.rect;

	/**
	 * Draws a rounded rectangle with all corners with the specified radius.
	 * @method drawRoundRect
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Number} w
	 * @param {Number} h
	 * @param {Number} radius Corner radius.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.drawRoundRect = function(x, y, w, h, radius) {
		this.drawRoundRectComplex(x, y, w, h, radius, radius, radius, radius);
		return this;
	};

	/**
	 * Draws a rounded rectangle with different corner radii. Supports positive and negative corner radii. A tiny API
	 * method "rc" also exists.
	 * @method drawRoundRectComplex
	 * @param {Number} x The horizontal coordinate to draw the round rect.
	 * @param {Number} y The vertical coordinate to draw the round rect.
	 * @param {Number} w The width of the round rect.
	 * @param {Number} h The height of the round rect.
	 * @param {Number} radiusTL Top left corner radius.
	 * @param {Number} radiusTR Top right corner radius.
	 * @param {Number} radiusBR Bottom right corner radius.
	 * @param {Number} radiusBL Bottom left corner radius.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.drawRoundRectComplex = function(x, y, w, h, radiusTL, radiusTR, radiusBR, radiusBL) {
		var max = (w<h?w:h)/2;
		var mTL=0, mTR=0, mBR=0, mBL=0;
		if (radiusTL < 0) { radiusTL *= (mTL=-1); }
		if (radiusTL > max) { radiusTL = max; }
		if (radiusTR < 0) { radiusTR *= (mTR=-1); }
		if (radiusTR > max) { radiusTR = max; }
		if (radiusBR < 0) { radiusBR *= (mBR=-1); }
		if (radiusBR > max) { radiusBR = max; }
		if (radiusBL < 0) { radiusBL *= (mBL=-1); }
		if (radiusBL > max) { radiusBL = max; }

		this._dirty = this._active = true;
		var arcTo=this._ctx.arcTo, lineTo=this._ctx.lineTo;
		this._activeInstructions.push(
			new Command(this._ctx.moveTo, [x+w-radiusTR, y]),
			new Command(arcTo, [x+w+radiusTR*mTR, y-radiusTR*mTR, x+w, y+radiusTR, radiusTR]),
			new Command(lineTo, [x+w, y+h-radiusBR]),
			new Command(arcTo, [x+w+radiusBR*mBR, y+h+radiusBR*mBR, x+w-radiusBR, y+h, radiusBR]),
			new Command(lineTo, [x+radiusBL, y+h]),
			new Command(arcTo, [x-radiusBL*mBL, y+h+radiusBL*mBL, x, y+h-radiusBL, radiusBL]),
			new Command(lineTo, [x, y+radiusTL]),
			new Command(arcTo, [x-radiusTL*mTL, y-radiusTL*mTL, x+radiusTL, y, radiusTL]),
			new Command(this._ctx.closePath)
		);
		return this;
	};

	/**
	 * Draws a circle with the specified radius at (x, y).
	 *
	 *      var g = new createjs.Graphics();
	 *	    g.setStrokeStyle(1);
	 *	    g.beginStroke(createjs.Graphics.getRGB(0,0,0));
	 *	    g.beginFill(createjs.Graphics.getRGB(255,0,0));
	 *	    g.drawCircle(0,0,3);
	 *
	 *	    var s = new createjs.Shape(g);
	 *		s.x = 100;
	 *		s.y = 100;
	 *
	 *	    stage.addChild(s);
	 *	    stage.update();
	 *
	 * A tiny API method "dc" also exists.
	 * @method drawCircle
	 * @param {Number} x x coordinate center point of circle.
	 * @param {Number} y y coordinate center point of circle.
	 * @param {Number} radius Radius of circle.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.drawCircle = function(x, y, radius) {
		this.arc(x, y, radius, 0, Math.PI*2);
		return this;
	};

	/**
	 * Draws an ellipse (oval) with a specified width (w) and height (h). Similar to {{#crossLink "Graphics/drawCircle"}}{{/crossLink}},
	 * except the width and height can be different. A tiny API method "de" also exists.
	 * @method drawEllipse
	 * @param {Number} x The left coordinate point of the ellipse. Note that this is different from {{#crossLink "Graphics/drawCircle"}}{{/crossLink}}
	 * which draws from center.
	 * @param {Number} y The top coordinate point of the ellipse. Note that this is different from {{#crossLink "Graphics/drawCircle"}}{{/crossLink}}
	 * which draws from the center.
	 * @param {Number} w The height (horizontal diameter) of the ellipse. The horizontal radius will be half of this
	 * number.
	 * @param {Number} h The width (vertical diameter) of the ellipse. The vertical radius will be half of this number.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.drawEllipse = function(x, y, w, h) {
		this._dirty = this._active = true;
		var k = 0.5522848;
		var ox = (w / 2) * k;
		var oy = (h / 2) * k;
		var xe = x + w;
		var ye = y + h;
		var xm = x + w / 2;
		var ym = y + h / 2;

		this._activeInstructions.push(
			new Command(this._ctx.moveTo, [x, ym]),
			new Command(this._ctx.bezierCurveTo, [x, ym-oy, xm-ox, y, xm, y]),
			new Command(this._ctx.bezierCurveTo, [xm+ox, y, xe, ym-oy, xe, ym]),
			new Command(this._ctx.bezierCurveTo, [xe, ym+oy, xm+ox, ye, xm, ye]),
			new Command(this._ctx.bezierCurveTo, [xm-ox, ye, x, ym+oy, x, ym])
		);
		return this;
	};

	/**
	 * Provides a method for injecting arbitrary Context2D (aka Canvas) API calls into a Graphics queue. The specified
	 * callback function will be called in sequence with other drawing instructions. The callback will be executed in the
	 * scope of the target canvas's Context2D object, and will be passed the data object as a parameter.
	 *
	 * This is an advanced feature. It can allow for powerful functionality, like injecting output from tools that
	 * export Context2D instructions, executing raw canvas calls within the context of the display list, or dynamically
	 * modifying colors or stroke styles within a Graphics instance over time, but it is not intended for general use.
	 *
	 * Within a Graphics queue, each path begins by applying the fill and stroke styles and settings, followed by
	 * drawing instructions, followed by the fill() and/or stroke() commands. This means that within a path, inject() can
	 * update the fill & stroke styles, but for it to be applied in a predictable manner, you must have begun a fill or
	 * stroke (as appropriate) normally via the Graphics API. For example:
	 *
	 * 	function setColor(color) {
	 * 		this.fillStyle = color;
	 * 	}
	 *
	 * 	// this will not draw anything - no fill was begun, so fill() is not called:
	 * 	myGraphics.inject(setColor, "red").drawRect(0,0,100,100);
	 *
	 * 	// this will draw the rect in green:
	 * 	myGraphics.beginFill("#000").inject(setColor, "green").drawRect(0,0,100,100);
	 *
	 * 	// this will draw both rects in blue, because there is only a single path
	 * 	// so the second inject overwrites the first:
	 * 	myGraphics.beginFill("#000").inject(setColor, "green").drawRect(0,0,100,100)
	 * 		.inject(setColor, "blue").drawRect(100,0,100,100);
	 *
	 * 	// this will draw the first rect in green, and the second in blue:
	 * 	myGraphics.beginFill("#000").inject(setColor, "green").drawRect(0,0,100,100)
	 * 		.beginFill("#000").inject(setColor, "blue").drawRect(100,0,100,100);
	 *
	 * @method inject
	 * @param {Function} callback The function to execute.
	 * @param {Object} data Arbitrary data that will be passed to the callback when it is executed.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.inject = function(callback, data) {
		this._dirty = this._active = true;

		this._activeInstructions.push(
			new Command(callback, [data])
		);
		return this;
	};

	/**
	 * Draws a star if pointSize is greater than 0, or a regular polygon if pointSize is 0 with the specified number of
	 * points. For example, the following code will draw a familiar 5 pointed star shape centered at 100, 100 and with a
	 * radius of 50:
	 *
	 *      myGraphics.beginFill("#FF0").drawPolyStar(100, 100, 50, 5, 0.6, -90);
	 *      // Note: -90 makes the first point vertical
	 *
	 * A tiny API method "dp" also exists.
	 *
	 * @method drawPolyStar
	 * @param {Number} x Position of the center of the shape.
	 * @param {Number} y Position of the center of the shape.
	 * @param {Number} radius The outer radius of the shape.
	 * @param {Number} sides The number of points on the star or sides on the polygon.
	 * @param {Number} pointSize The depth or "pointy-ness" of the star points. A pointSize of 0 will draw a regular
	 * polygon (no points), a pointSize of 1 will draw nothing because the points are infinitely pointy.
	 * @param {Number} angle The angle of the first point / corner. For example a value of 0 will draw the first point
	 * directly to the right of the center.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.drawPolyStar = function(x, y, radius, sides, pointSize, angle) {
		this._dirty = this._active = true;
		if (pointSize == null) { pointSize = 0; }
		pointSize = 1-pointSize;
		if (angle == null) { angle = 0; }
		else { angle /= 180/Math.PI; }
		var a = Math.PI/sides;

		this._activeInstructions.push(new Command(this._ctx.moveTo, [x+Math.cos(angle)*radius, y+Math.sin(angle)*radius]));
		for (var i=0; i<sides; i++) {
			angle += a;
			if (pointSize != 1) {
				this._activeInstructions.push(new Command(this._ctx.lineTo, [x+Math.cos(angle)*radius*pointSize, y+Math.sin(angle)*radius*pointSize]));
			}
			angle += a;
			this._activeInstructions.push(new Command(this._ctx.lineTo, [x+Math.cos(angle)*radius, y+Math.sin(angle)*radius]));
		}
		return this;
	};

	/**
	 * Decodes a compact encoded path string into a series of draw instructions.
	 * This format is not intended to be human readable, and is meant for use by authoring tools.
	 * The format uses a base64 character set, with each character representing 6 bits, to define a series of draw
	 * commands.
	 *
	 * Each command is comprised of a single "header" character followed by a variable number of alternating x and y
	 * position values. Reading the header bits from left to right (most to least significant): bits 1 to 3 specify the
	 * type of operation (0-moveTo, 1-lineTo, 2-quadraticCurveTo, 3-bezierCurveTo, 4-closePath, 5-7 unused). Bit 4
	 * indicates whether position values use 12 bits (2 characters) or 18 bits (3 characters), with a one indicating the
	 * latter. Bits 5 and 6 are currently unused.
	 *
	 * Following the header is a series of 0 (closePath), 2 (moveTo, lineTo), 4 (quadraticCurveTo), or 6 (bezierCurveTo)
	 * parameters. These parameters are alternating x/y positions represented by 2 or 3 characters (as indicated by the
	 * 4th bit in the command char). These characters consist of a 1 bit sign (1 is negative, 0 is positive), followed
	 * by an 11 (2 char) or 17 (3 char) bit integer value. All position values are in tenths of a pixel. Except in the
	 * case of move operations which are absolute, this value is a delta from the previous x or y position (as
	 * appropriate).
	 *
	 * For example, the string "A3cAAMAu4AAA" represents a line starting at -150,0 and ending at 150,0.
	 * <br />A - bits 000000. First 3 bits (000) indicate a moveTo operation. 4th bit (0) indicates 2 chars per
	 * parameter.
	 * <br />n0 - 110111011100. Absolute x position of -150.0px. First bit indicates a negative value, remaining bits
	 * indicate 1500 tenths of a pixel.
	 * <br />AA - 000000000000. Absolute y position of 0.
	 * <br />I - 001100. First 3 bits (001) indicate a lineTo operation. 4th bit (1) indicates 3 chars per parameter.
	 * <br />Au4 - 000000101110111000. An x delta of 300.0px, which is added to the previous x value of -150.0px to
	 * provide an absolute position of +150.0px.
	 * <br />AAA - 000000000000000000. A y delta value of 0.
	 *
	 * A tiny API method "p" also exists.
	 * @method decodePath
	 * @param {String} str The path string to decode.
	 * @return {Graphics} The Graphics instance the method is called on (useful for chaining calls.)
	 **/
	p.decodePath = function(str) {
		var instructions = [this.moveTo, this.lineTo, this.quadraticCurveTo, this.bezierCurveTo, this.closePath];
		var paramCount = [2, 2, 4, 6, 0];
		var i=0, l=str.length;
		var params = [];
		var x=0, y=0;
		var base64 = Graphics.BASE_64;

		while (i<l) {
			var c = str.charAt(i);
			var n = base64[c];
			var fi = n>>3; // highest order bits 1-3 code for operation.
			var f = instructions[fi];
			// check that we have a valid instruction & that the unused bits are empty:
			if (!f || (n&3)) { throw("bad path data (@"+i+"): "+c); }
			var pl = paramCount[fi];
			if (!fi) { x=y=0; } // move operations reset the position.
			params.length = 0;
			i++;
			var charCount = (n>>2&1)+2;  // 4th header bit indicates number size for this operation.
			for (var p=0; p<pl; p++) {
				var num = base64[str.charAt(i)];
				var sign = (num>>5) ? -1 : 1;
				num = ((num&31)<<6)|(base64[str.charAt(i+1)]);
				if (charCount == 3) { num = (num<<6)|(base64[str.charAt(i+2)]); }
				num = sign*num/10;
				if (p%2) { x = (num += x); }
				else { y = (num += y); }
				params[p] = num;
				i += charCount;
			}
			f.apply(this,params);
		}
		return this;
	};

	/**
	 * Returns a clone of this Graphics instance.
	 * @method clone
	 * @return {Graphics} A clone of the current Graphics instance.
	 **/
	p.clone = function() {
		var o = new Graphics();
		o._instructions = this._instructions.slice();
		o._activeInstructions = this._activeInstructions.slice();
		o._oldInstructions = this._oldInstructions.slice();
		if (this._fillInstructions) { o._fillInstructions = this._fillInstructions.slice(); }
		if (this._strokeInstructions) { o._strokeInstructions = this._strokeInstructions.slice(); }
		if (this._strokeStyleInstructions) { o._strokeStyleInstructions = this._strokeStyleInstructions.slice(); }
		o._active = this._active;
		o._dirty = this._dirty;
		o._fillMatrix = this._fillMatrix;
		o._strokeIgnoreScale = this._strokeIgnoreScale;
		return o;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Graphics]";
	};


// tiny API:
	/** Shortcut to moveTo.
	 * @method mt
	 * @protected
	 * @type {Function}
	 **/
	p.mt = p.moveTo;

	/** Shortcut to lineTo.
	 * @method lt
	 * @protected
	 * @type {Function}
	 **/
	p.lt = p.lineTo;

	/** Shortcut to arcTo.
	 * @method at
	 * @protected
	 * @type {Function}
	 **/
	p.at = p.arcTo;

	/** Shortcut to bezierCurveTo.
	 * @method bt
	 * @protected
	 * @type {Function}
	 **/
	p.bt = p.bezierCurveTo;

	/** Shortcut to quadraticCurveTo / curveTo.
	 * @method qt
	 * @protected
	 * @type {Function}
	 **/
	p.qt = p.quadraticCurveTo;

	/** Shortcut to arc.
	 * @method a
	 * @protected
	 * @type {Function}
	 **/
	p.a = p.arc;

	/** Shortcut to rect.
	 * @method r
	 * @protected
	 * @type {Function}
	 **/
	p.r = p.rect;

	/** Shortcut to closePath.
	 * @method cp
	 * @protected
	 * @type {Function}
	 **/
	p.cp = p.closePath;

	/** Shortcut to clear.
	 * @method c
	 * @protected
	 * @type {Function}
	 **/
	p.c = p.clear;

	/** Shortcut to beginFill.
	 * @method f
	 * @protected
	 * @type {Function}
	 **/
	p.f = p.beginFill;

	/** Shortcut to beginLinearGradientFill.
	 * @method lf
	 * @protected
	 * @type {Function}
	 **/
	p.lf = p.beginLinearGradientFill;

	/** Shortcut to beginRadialGradientFill.
	 * @method rf
	 * @protected
	 * @type {Function}
	 **/
	p.rf = p.beginRadialGradientFill;

	/** Shortcut to beginBitmapFill.
	 * @method bf
	 * @protected
	 * @type {Function}
	 **/
	p.bf = p.beginBitmapFill;

	/** Shortcut to endFill.
	 * @method ef
	 * @protected
	 * @type {Function}
	 **/
	p.ef = p.endFill;

	/** Shortcut to setStrokeStyle.
	 * @method ss
	 * @protected
	 * @type {Function}
	 **/
	p.ss = p.setStrokeStyle;

	/** Shortcut to beginStroke.
	 * @method s
	 * @protected
	 * @type {Function}
	 **/
	p.s = p.beginStroke;

	/** Shortcut to beginLinearGradientStroke.
	 * @method ls
	 * @protected
	 * @type {Function}
	 **/
	p.ls = p.beginLinearGradientStroke;

	/** Shortcut to beginRadialGradientStroke.
	 * @method rs
	 * @protected
	 * @type {Function}
	 **/
	p.rs = p.beginRadialGradientStroke;

	/** Shortcut to beginBitmapStroke.
	 * @method bs
	 * @protected
	 * @type {Function}
	 **/
	p.bs = p.beginBitmapStroke;

	/** Shortcut to endStroke.
	 * @method es
	 * @protected
	 * @type {Function}
	 **/
	p.es = p.endStroke;

	/** Shortcut to drawRect.
	 * @method dr
	 * @protected
	 * @type {Function}
	 **/
	p.dr = p.drawRect;

	/** Shortcut to drawRoundRect.
	 * @method rr
	 * @protected
	 * @type {Function}
	 **/
	p.rr = p.drawRoundRect;

	/** Shortcut to drawRoundRectComplex.
	 * @method rc
	 * @protected
	 * @type {Function}
	 **/
	p.rc = p.drawRoundRectComplex;

	/** Shortcut to drawCircle.
	 * @method dc
	 * @protected
	 * @type {Function}
	 **/
	p.dc = p.drawCircle;

	/** Shortcut to drawEllipse.
	 * @method de
	 * @protected
	 * @type {Function}
	 **/
	p.de = p.drawEllipse;

	/** Shortcut to drawPolyStar.
	 * @method dp
	 * @protected
	 * @type {Function}
	 **/
	p.dp = p.drawPolyStar;

	/** Shortcut to decodePath.
	 * @method p
	 * @protected
	 * @type Function
	 **/
	p.p = p.decodePath;


// private methods:
	/**
	 * @method _updateInstructions
	 * @protected
	 **/
	p._updateInstructions = function() {
		this._instructions = this._oldInstructions.slice();
		this._instructions.push(Graphics.beginCmd);

		this._appendInstructions(this._fillInstructions);
		this._appendInstructions(this._strokeInstructions);
		this._appendInstructions(this._strokeInstructions&&this._strokeStyleInstructions);

		this._appendInstructions(this._activeInstructions);

		if (this._fillInstructions) {
			this._appendDraw(Graphics.fillCmd, this._fillMatrix);
		}
		if (this._strokeInstructions) {
			this._appendDraw(Graphics.strokeCmd, this._strokeIgnoreScale&&[1,0,0,1,0,0]);
		}
	};

	/**
	 * @method _appendInstructions
	 * @protected
	 **/
	p._appendInstructions = function(instructions) {
		if (instructions) { this._instructions.push.apply(this._instructions, instructions); }
	};

	/**
	 * @method _appendDraw
	 * @protected
	 **/
	p._appendDraw = function(command, matrixArr) {
		if (!matrixArr) { this._instructions.push(command); }
		else {
			this._instructions.push(
				new Command(this._ctx.save, [], false),
				new Command(this._ctx.transform, matrixArr, false),
				command,
				new Command(this._ctx.restore, [], false)
			);
		}
	};

	/**
	 * @method _newPath
	 * @protected
	 **/
	p._newPath = function() {
		if (this._dirty) { this._updateInstructions(); }
		this._oldInstructions = this._instructions;
		this._activeInstructions = [];
		this._active = this._dirty = false;
	};

	// used to create Commands that set properties:
	/**
	 * Used to create Commands that set properties
	 * @method _setProp
	 * @param {String} name
	 * @param {String} value
	 * @protected
	 **/
	p._setProp = function(name, value) {
		this[name] = value;
	};

createjs.Graphics = Graphics;
}());
/*
* DisplayObject
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
* 
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
* 
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * The EaselJS Javascript library provides a retained graphics mode for canvas including a full hierarchical display
 * list, a core interaction model, and helper classes to make working with 2D graphics in Canvas much easier.
 * EaselJS provides straight forward solutions for working with rich graphics and interactivity with HTML5 Canvas...
 *
 * <h4>Getting Started</h4>
 * To get started with Easel, create a {{#crossLink "Stage"}}{{/crossLink}} that wraps a CANVAS element, and add
 * {{#crossLink "DisplayObject"}}{{/crossLink}} instances as children. EaselJS supports:
 * <ul>
 *      <li>Images using {{#crossLink "Bitmap"}}{{/crossLink}}</li>
 *      <li>Vector graphics using {{#crossLink "Shape"}}{{/crossLink}} and {{#crossLink "Graphics"}}{{/crossLink}}</li>
 *      <li>Animated bitmaps using {{#crossLink "SpriteSheet"}}{{/crossLink}} and {{#crossLink "Sprite"}}{{/crossLink}}
 *      <li>Simple text instances using {{#crossLink "Text"}}{{/crossLink}}</li>
 *      <li>Containers that hold other DisplayObjects using {{#crossLink "Container"}}{{/crossLink}}</li>
 *      <li>Control HTML DOM elements using {{#crossLink "DOMElement"}}{{/crossLink}}</li>
 * </ul>
 *
 * All display objects can be added to the stage as children, or drawn to a canvas directly.
 *
 * <b>User Interactions</b><br />
 * All display objects on stage (except DOMElement) will dispatch events when interacted with using a mouse or
 * touch. EaselJS supports hover, press, and release events, as well as an easy-to-use drag-and-drop model. Check out
 * {{#crossLink "MouseEvent"}}{{/crossLink}} for more information.
 *
 * <h4>Simple Example</h4>
 * This example illustrates how to create and position a {{#crossLink "Shape"}}{{/crossLink}} on the {{#crossLink "Stage"}}{{/crossLink}}
 * using EaselJS' drawing API.
 *
 *	    //Create a stage by getting a reference to the canvas
 *	    stage = new createjs.Stage("demoCanvas");
 *	    //Create a Shape DisplayObject.
 *	    circle = new createjs.Shape();
 *	    circle.graphics.beginFill("red").drawCircle(0, 0, 40);
 *	    //Set position of Shape instance.
 *	    circle.x = circle.y = 50;
 *	    //Add Shape instance to stage display list.
 *	    stage.addChild(circle);
 *	    //Update stage will render next frame
 *	    stage.update();
 *
 * <b>Simple Interaction Example</b><br>
 *
 *      displayObject.addEventListener("click", handleClick);
 *      function handleClick(event){
 *          // Click happenened
 *      }
 *
 *      displayObject.addEventListener("mousedown", handlePress);
 *      function handlePress(event) {
 *          // A mouse press happened.
 *          // Listen for mouse move while the mouse is down:
 *          event.addEventListener("mousemove", handleMove);
 *      }
 *      function handleMove(event) {
 *          // Check out the DragAndDrop example in GitHub for more
 *      }
 *
 * <b>Simple Animation Example</b><br />
 * This example moves the shape created in the previous demo across the screen.
 *
 *	    //Update stage will render next frame
 *	    createjs.Ticker.addEventListener("tick", handleTick);
 *
 *	    function handleTick() {
 *          //Circle will move 10 units to the right.
 *	    	circle.x += 10;
 *	    	//Will cause the circle to wrap back
 * 	    	if (circle.x > stage.canvas.width) { circle.x = 0; }
 *	    	stage.update();
 *	    }
 *
 * <h4>Other Features</h4>
 * EaselJS also has built in support for
 * <ul><li>Canvas features such as {{#crossLink "Shadow"}}{{/crossLink}} and CompositeOperation</li>
 *      <li>{{#crossLink "Ticker"}}{{/crossLink}}, a global heartbeat that objects can subscribe to</li>
 *      <li>Filters, including a provided {{#crossLink "ColorMatrixFilter"}}{{/crossLink}}, {{#crossLink "AlphaMaskFilter"}}{{/crossLink}},
 *      {{#crossLink "AlphaMapFilter"}}{{/crossLink}}, and {{#crossLink "BlurFilter"}}{{/crossLink}}. See {{#crossLink "Filter"}}{{/crossLink}}
 *      for more information</li>
 *      <li>A {{#crossLink "ButtonHelper"}}{{/crossLink}} utility, to easily create interactive buttons</li>
 *      <li>{{#crossLink "SpriteSheetUtils"}}{{/crossLink}} and a {{#crossLink "SpriteSheetBuilder"}}{{/crossLink}} to
 *      help build and manage {{#crossLink "SpriteSheet"}}{{/crossLink}} functionality at run-time.</li>
 * </ul>
 *
 * <h4>Browser Support</h4>
 * All modern browsers that support Canvas will support EaselJS (<a href="http://caniuse.com/canvas">http://caniuse.com/canvas</a>).
 * Browser performance may vary between platforms, for example, Android Canvas has poor hardware support, and is much
 * slower on average than most other browsers.
 *
 * @module EaselJS
 * @main EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
/**
 * DisplayObject is an abstract class that should not be constructed directly. Instead construct subclasses such as
 * {{#crossLink "Container"}}{{/crossLink}}, {{#crossLink "Bitmap"}}{{/crossLink}}, and {{#crossLink "Shape"}}{{/crossLink}}.
 * DisplayObject is the base class for all display classes in the EaselJS library. It defines the core properties and
 * methods that are shared between all display objects, such as transformation properties (x, y, scaleX, scaleY, etc),
 * caching, and mouse handlers.
 * @class DisplayObject
 * @extends EventDispatcher
 * @constructor
 **/
var DisplayObject = function() {
  this.initialize();
};
var p = DisplayObject.prototype = new createjs.EventDispatcher();
	
	/**
	 * Listing of mouse event names. Used in _hasMouseEventListener.
	 * @property _MOUSE_EVENTS
	 * @protected
	 * @static
	 * @type {Array}
	 **/
	DisplayObject._MOUSE_EVENTS = ["click","dblclick","mousedown","mouseout","mouseover","pressmove","pressup","rollout","rollover"];

	/**
	 * Suppresses errors generated when using features like hitTest, mouse events, and {{#crossLink "getObjectsUnderPoint"}}{{/crossLink}}
	 * with cross domain content.
	 * @property suppressCrossDomainErrors
	 * @static
	 * @type {Boolean}
	 * @default false
	 **/
	DisplayObject.suppressCrossDomainErrors = false;

	/**
	 * @property _hitTestCanvas
	 * @type {HTMLCanvasElement | Object}
	 * @static
	 * @protected
	 **/
	 
	/**
	 * @property _hitTestContext
	 * @type {CanvasRenderingContext2D}
	 * @static
	 * @protected
	 **/
	var canvas = createjs.createCanvas?createjs.createCanvas():document.createElement("canvas"); // prevent errors on load in browsers without canvas.
	if (canvas.getContext) {
		DisplayObject._hitTestCanvas = canvas;
		DisplayObject._hitTestContext = canvas.getContext("2d");
		canvas.width = canvas.height = 1;
	}

	/**
	 * @property _nextCacheID
	 * @type {Number}
	 * @static
	 * @protected
	 **/
	DisplayObject._nextCacheID = 1;

// events:

	/**
	 * Dispatched when the user presses their left mouse button over the display object. See the 
	 * {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event mousedown
	 * @since 0.6.0
	 */
	 
	/**
	 * Dispatched when the user presses their left mouse button and then releases it while over the display object.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event click
	 * @since 0.6.0
	 */
	 
	/**
	 * Dispatched when the user double clicks their left mouse button over this display object.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event dblclick
	 * @since 0.6.0
	 */
	 
	/**
	 * Dispatched when the user's mouse enters this display object. This event must be enabled using 
	 * {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}}. See also {{#crossLink "DisplayObject/rollover:event"}}{{/crossLink}}.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event mouseover
	 * @since 0.6.0
	 */

	/**
	 * Dispatched when the user's mouse leaves this display object. This event must be enabled using 
	 * {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}}. See also {{#crossLink "DisplayObject/rollout:event"}}{{/crossLink}}.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event mouseout
	 * @since 0.6.0
	 */
	 
	/**
	 * This event is similar to {{#crossLink "DisplayObject/mouseover:event"}}{{/crossLink}}, with the following
	 * differences: it does not bubble, and it considers {{#crossLink "Container"}}{{/crossLink}} instances as an
	 * aggregate of their content.
	 * 
	 * For example, myContainer contains two overlapping children: shapeA and shapeB. The user moves their mouse over
	 * shapeA and then directly on to shapeB. With a listener for {{#crossLink "mouseover:event"}}{{/crossLink}} on
	 * myContainer, two events would be received, each targeting a child element:<OL>
	 * <LI>when the mouse enters shapeA (target=shapeA)</LI>
	 * <LI>when the mouse enters shapeB (target=shapeB)</LI>
	 * </OL>
	 * However, with a listener for "rollover" instead, only a single event is received when the mouse first enters
	 * the aggregate myContainer content (target=myContainer).
	 * 
	 * This event must be enabled using {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}}.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event rollover
	 * @since 0.7.0
	 */
	 
	/**
	 * This event is similar to {{#crossLink "DisplayObject/mouseout:event"}}{{/crossLink}}, with the following
	 * differences: it does not bubble, and it considers {{#crossLink "Container"}}{{/crossLink}} instances as an
	 * aggregate of their content.
	 * 
	 * For example, myContainer contains two overlapping children: shapeA and shapeB. The user moves their mouse over
	 * shapeA, then directly on to shapeB, then off both. With a listener for {{#crossLink "mouseout:event"}}{{/crossLink}}
	 * on myContainer, two events would be received, each targeting a child element:<OL>
	 * <LI>when the mouse leaves shapeA (target=shapeA)</LI>
	 * <LI>when the mouse leaves shapeB (target=shapeB)</LI>
	 * </OL>
	 * However, with a listener for "rollout" instead, only a single event is received when the mouse leaves
	 * the aggregate myContainer content (target=myContainer).
	 * 
	 * This event must be enabled using {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}}.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event rollout
	 * @since 0.7.0
	 */
	 
	/**
	 * After a {{#crossLink "DisplayObject/mousedown:event"}}{{/crossLink}} occurs on a display object, a pressmove
	 * event will be generated on that object whenever the mouse moves until the mouse press is released. This can be
	 * useful for dragging and similar operations.
	 * @event pressmove
	 * @since 0.7.0
	 */
	 
	/**
	 * After a {{#crossLink "DisplayObject/mousedown:event"}}{{/crossLink}} occurs on a display object, a pressup event
	 * will be generated on that object when that mouse press is released. This can be useful for dragging and similar
	 * operations.
	 * @event pressup
	 * @since 0.7.0
	 */
	 
	/**
	 * Dispatched on each display object on a stage whenever the stage updates. This occurs immediately before the
	 * rendering (draw) pass. When {{#crossLink "Stage/update"}}{{/crossLink}} is called, first all display objects on
	 * the stage dispatch the tick event, then all of the display objects are drawn to stage. Children will have their
	 * {{#crossLink "tick:event"}}{{/crossLink}} event dispatched in order of their depth prior to the event being
	 * dispatched on their parent.
	 * @event tick
	 * @param {Object} target The object that dispatched the event.
	 * @param {String} type The event type.
	 * @param {Array} params An array containing any arguments that were passed to the Stage.update() method. For
	 *      example if you called stage.update("hello"), then the params would be ["hello"].
	 * @since 0.6.0
	 */

// public properties:
	/**
	 * The alpha (transparency) for this display object. 0 is fully transparent, 1 is fully opaque.
	 * @property alpha
	 * @type {Number}
	 * @default 1
	 **/
	p.alpha = 1;

	/**
	 * If a cache is active, this returns the canvas that holds the cached version of this display object. See {{#crossLink "cache"}}{{/crossLink}}
	 * for more information.
	 * @property cacheCanvas
	 * @type {HTMLCanvasElement | Object}
	 * @default null
	 * @readonly
	 **/
	p.cacheCanvas = null;

	/**
	 * Unique ID for this display object. Makes display objects easier for some uses.
	 * @property id
	 * @type {Number}
	 * @default -1
	 **/
	p.id = -1;

	/**
	 * Indicates whether to include this object when running mouse interactions. Setting this to `false` for children
	 * of a {{#crossLink "Container"}}{{/crossLink}} will cause events on the Container to not fire when that child is
	 * clicked. Setting this property to `false` does not prevent the {{#crossLink "Container/getObjectsUnderPoint"}}{{/crossLink}}
	 * method from returning the child.
	 *
	 * <strong>Note:</strong> In EaselJS 0.7.0, the mouseEnabled property will not work properly with nested Containers. Please
	 * check out the latest NEXT version in <a href="https://github.com/CreateJS/EaselJS/tree/master/lib">GitHub</a> for an updated version with this issue resolved. The fix will be
	 * provided in the next release of EaselJS.
	 * @property mouseEnabled
	 * @type {Boolean}
	 * @default true
	 **/
	p.mouseEnabled = true;
	
	/**
	 * If false, the tick will not run on this display object (or its children). This can provide some performance benefits.
	 * In addition to preventing the "tick" event from being dispatched, it will also prevent tick related updates
	 * on some display objects (ex. Sprite & MovieClip frame advancing, DOMElement visibility handling).
	 * @property tickEnabled
	 * @type Boolean
	 * @default true
	 **/
	p.tickEnabled = true;

	/**
	 * An optional name for this display object. Included in {{#crossLink "DisplayObject/toString"}}{{/crossLink}} . Useful for
	 * debugging.
	 * @property name
	 * @type {String}
	 * @default null
	 **/
	p.name = null;

	/**
	 * A reference to the {{#crossLink "Container"}}{{/crossLink}} or {{#crossLink "Stage"}}{{/crossLink}} object that
	 * contains this display object, or null if it has not been added
	 * to one.
	 * @property parent
	 * @final
	 * @type {Container}
	 * @default null
	 * @readonly
	 **/
	p.parent = null;

	/**
	 * The left offset for this display object's registration point. For example, to make a 100x100px Bitmap rotate
	 * around its center, you would set regX and {{#crossLink "DisplayObject/regY:property"}}{{/crossLink}} to 50.
	 * @property regX
	 * @type {Number}
	 * @default 0
	 **/
	p.regX = 0;

	/**
	 * The y offset for this display object's registration point. For example, to make a 100x100px Bitmap rotate around
	 * its center, you would set {{#crossLink "DisplayObject/regX:property"}}{{/crossLink}} and regY to 50.
	 * @property regY
	 * @type {Number}
	 * @default 0
	 **/
	p.regY = 0;

	/**
	 * The rotation in degrees for this display object.
	 * @property rotation
	 * @type {Number}
	 * @default 0
	 **/
	p.rotation = 0;

	/**
	 * The factor to stretch this display object horizontally. For example, setting scaleX to 2 will stretch the display
	 * object to twice its nominal width. To horizontally flip an object, set the scale to a negative number.
	 * @property scaleX
	 * @type {Number}
	 * @default 1
	 **/
	p.scaleX = 1;

	/**
	 * The factor to stretch this display object vertically. For example, setting scaleY to 0.5 will stretch the display
	 * object to half its nominal height. To vertically flip an object, set the scale to a negative number.
	 * @property scaleY
	 * @type {Number}
	 * @default 1
	 **/
	p.scaleY = 1;

	/**
	 * The factor to skew this display object horizontally.
	 * @property skewX
	 * @type {Number}
	 * @default 0
	 **/
	p.skewX = 0;

	/**
	 * The factor to skew this display object vertically.
	 * @property skewY
	 * @type {Number}
	 * @default 0
	 **/
	p.skewY = 0;

	/**
	 * A shadow object that defines the shadow to render on this display object. Set to `null` to remove a shadow. If
	 * null, this property is inherited from the parent container.
	 * @property shadow
	 * @type {Shadow}
	 * @default null
	 **/
	p.shadow = null;

	/**
	 * Indicates whether this display object should be rendered to the canvas and included when running the Stage
	 * {{#crossLink "Stage/getObjectsUnderPoint"}}{{/crossLink}} method.
	 * @property visible
	 * @type {Boolean}
	 * @default true
	 **/
	p.visible = true;

	/**
	 * The x (horizontal) position of the display object, relative to its parent.
	 * @property x
	 * @type {Number}
	 * @default 0
	 **/
	p.x = 0;

	/** The y (vertical) position of the display object, relative to its parent.
	 * @property y
	 * @type {Number}
	 * @default 0
	 **/
	p.y = 0;

	/**
	 * The composite operation indicates how the pixels of this display object will be composited with the elements
	 * behind it. If `null`, this property is inherited from the parent container. For more information, read the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#compositing">
	 * whatwg spec on compositing</a>.
	 * @property compositeOperation
	 * @type {String}
	 * @default null
	 **/
	p.compositeOperation = null;

	/**
	 * Indicates whether the display object should have its x & y position rounded prior to drawing it to stage.
	 * Snapping to whole pixels can result in a sharper and faster draw for images (ex. Bitmap & cached objects).
	 * This only applies if the enclosing stage has {{#crossLink "Stage/snapPixelsEnabled:property"}}{{/crossLink}} set
	 * to `true`. The snapToPixel property is `true` by default for {{#crossLink "Bitmap"}}{{/crossLink}} and {{#crossLink "Sprite"}}{{/crossLink}}
	 * instances, and `false` for all other display objects.
	 *
	 * Note that this applies only rounds the display object's local position. You should ensure that all of the display
	 * object's ancestors (parent containers) are also on a whole pixel. You can do this by setting the ancestors'
	 * snapToPixel property to `true`.
	 * @property snapToPixel
	 * @type {Boolean}
	 * @default false
	 * @deprecated Hardware acceleration in modern browsers makes this unnecessary.
	 **/
	p.snapToPixel = false;
	
	// TODO: remove handler docs in future:
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "DisplayObject/mousedown:event"}}{{/crossLink}}
	 * event.
	 * @property onPress
	 * @type {Function}
	 * @deprecated Use addEventListener and the "mousedown" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "DisplayObject/click:event"}}{{/crossLink}}
	 * event.
	 * @property onClick
	 * @type {Function}
	 * @deprecated Use addEventListener and the "click" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "DisplayObject/dblclick:event"}}{{/crossLink}}
	 * event.
	 * @property onDoubleClick
	 * @type {Function}
	 * @deprecated Use addEventListener and the "dblclick" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "DisplayObject/mouseover:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseOver
	 * @type {Function}
	 * @deprecated Use addEventListener and the "mouseover" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "DisplayObject/mouseout:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseOut
	 * @type {Function}
	 * @deprecated Use addEventListener and the "mouseout" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "DisplayObject/tick:event"}}{{/crossLink}}
	 * event.
	 * @property onTick
	 * @type {Function}
	 * @deprecatedtick
	 */

	/**
	 * An array of Filter objects to apply to this display object. Filters are only applied / updated when {{#crossLink "cache"}}{{/crossLink}}
	 * or {{#crossLink "updateCache"}}{{/crossLink}} is called on the display object, and only apply to the area that is
	 * cached.
	 * @property filters
	 * @type {Array}
	 * @default null
	 **/
	p.filters = null;

	/**
	 * Returns an ID number that uniquely identifies the current cache for this display object. This can be used to
	 * determine if the cache has changed since a previous check.
	 * @property cacheID
	 * @type {Number}
	 * @default 0
	 */
	p.cacheID = 0;
	
	/**
	 * A Shape instance that defines a vector mask (clipping path) for this display object.  The shape's transformation
	 * will be applied relative to the display object's parent coordinates (as if it were a child of the parent).
	 * @property mask
	 * @type {Shape}
	 * @default null
	 */
	p.mask = null;
	
	/**
	 * A display object that will be tested when checking mouse interactions or testing {{#crossLink "Container/getObjectsUnderPoint"}}{{/crossLink}}.
	 * The hit area will have its transformation applied relative to this display object's coordinate space (as though
	 * the hit test object were a child of this display object and relative to its regX/Y). The hitArea will be tested
	 * using only its own `alpha` value regardless of the alpha value on the target display object, or the target's
	 * ancestors (parents).
	 * 
	 * If set on a {{#crossLink "Container"}}{{/crossLink}}, children of the Container will not receive mouse events.
	 * This is similar to setting {{#crossLink "mouseChildren"}}{{/crossLink}} to false.
	 *
	 * Note that hitArea is NOT currently used by the `hitTest()` method, nor is it supported for {{#crossLink "Stage"}}{{/crossLink}}.
	 * @property hitArea
	 * @type {DisplayObject}
	 * @default null
	 */
	p.hitArea = null;
	
	/**
	 * A CSS cursor (ex. "pointer", "help", "text", etc) that will be displayed when the user hovers over this display
	 * object. You must enable mouseover events using the {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}} method to
	 * use this property. Setting a non-null cursor on a Container will override the cursor set on its descendants.
	 * @property cursor
	 * @type {String}
	 * @default null
	 */
	p.cursor = null;

// private properties:

	/**
	 * @property _cacheOffsetX
	 * @protected
	 * @type {Number}
	 * @default 0
	 **/
	p._cacheOffsetX = 0;

	/**
	 * @property _cacheOffsetY
	 * @protected
	 * @type {Number}
	 * @default 0
	 **/
	p._cacheOffsetY = 0;
	
	/**
	 * @property _cacheScale
	 * @protected
	 * @type {Number}
	 * @default 1
	 **/
	p._cacheScale = 1;

	/**
	* @property _cacheDataURLID
	* @protected
	* @type {Number}
	* @default 0
	*/
	p._cacheDataURLID = 0;
	
	/**
	* @property _cacheDataURL
	* @protected
	* @type {String}
	* @default null
	*/
	p._cacheDataURL = null;

	/**
	 * @property _matrix
	 * @protected
	 * @type {Matrix2D}
	 * @default null
	 **/
	p._matrix = null;

	/**
	 * @property _rectangle
	 * @protected
	 * @type {Rectangle}
	 * @default null
	 **/
	p._rectangle = null;

	/**
	 * @property _bounds
	 * @protected
	 * @type {Rectangle}
	 * @default null
	 **/
	p._bounds = null;
	

// constructor:
	// separated so it can be easily addressed in subclasses:

	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	*/
	p.initialize = function() {
		this.id = createjs.UID.get();
		this._matrix = new createjs.Matrix2D();
		this._rectangle = new createjs.Rectangle();
	};

// public methods:
	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the display object would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0);
	};

	/**
	 * Draws the display object into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns <code>true</code> if the draw was handled (useful for overriding functionality).
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} [ignoreCache=false] Indicates whether the draw operation should ignore any current cache. For example,
	 * used for drawing the cache (to prevent it from simply drawing an existing cache back into itself).
	 * @return {Boolean}
	 **/
	p.draw = function(ctx, ignoreCache) {
		var cacheCanvas = this.cacheCanvas;
		if (ignoreCache || !cacheCanvas) { return false; }
		var scale = this._cacheScale, offX = this._cacheOffsetX, offY = this._cacheOffsetY, fBounds;
		if (fBounds = this._applyFilterBounds(offX, offY, 0, 0)) {
			offX = fBounds.x;
			offY = fBounds.y;
		}
		ctx.drawImage(cacheCanvas, offX, offY, cacheCanvas.width/scale, cacheCanvas.height/scale);
		return true;
	};
	
	/**
	 * Applies this display object's transformation, alpha, globalCompositeOperation, clipping path (mask), and shadow
	 * to the specified context. This is typically called prior to {{#crossLink "DisplayObject/draw"}}{{/crossLink}}.
	 * @method updateContext
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D to update.
	 **/
	p.updateContext = function(ctx) {
		var mtx, mask=this.mask, o=this;
		
		if (mask && mask.graphics && !mask.graphics.isEmpty()) {
			mtx = mask.getMatrix(mask._matrix);
			ctx.transform(mtx.a,  mtx.b, mtx.c, mtx.d, mtx.tx, mtx.ty);
			
			mask.graphics.drawAsPath(ctx);
			ctx.clip();
			
			mtx.invert();
			ctx.transform(mtx.a,  mtx.b, mtx.c, mtx.d, mtx.tx, mtx.ty);
		}
		
		mtx = o._matrix.identity().appendTransform(o.x, o.y, o.scaleX, o.scaleY, o.rotation, o.skewX, o.skewY, o.regX, o.regY);
		// TODO: should be a better way to manage this setting. For now, using dynamic access to avoid circular dependencies:
		if (createjs["Stage"]._snapToPixelEnabled && o.snapToPixel) { ctx.transform(mtx.a,  mtx.b, mtx.c, mtx.d, mtx.tx+0.5|0, mtx.ty+0.5|0); }
		else { ctx.transform(mtx.a,  mtx.b, mtx.c, mtx.d, mtx.tx, mtx.ty); }
		ctx.globalAlpha *= o.alpha;
		if (o.compositeOperation) { ctx.globalCompositeOperation = o.compositeOperation; }
		if (o.shadow) { this._applyShadow(ctx, o.shadow); }
	};

	/**
	 * Draws the display object into a new canvas, which is then used for subsequent draws. For complex content
	 * that does not change frequently (ex. a Container with many children that do not move, or a complex vector Shape),
	 * this can provide for much faster rendering because the content does not need to be re-rendered each tick. The
	 * cached display object can be moved, rotated, faded, etc freely, however if its content changes, you must
	 * manually update the cache by calling <code>updateCache()</code> or <code>cache()</code> again. You must specify
	 * the cache area via the x, y, w, and h parameters. This defines the rectangle that will be rendered and cached
	 * using this display object's coordinates.
	 *
	 * <h4>Example</h4>
	 * For example if you defined a Shape that drew a circle at 0, 0 with a radius of 25:
	 *
	 *      var shape = new createjs.Shape();
	 *      shape.graphics.beginFill("#ff0000").drawCircle(0, 0, 25);
	 *      myShape.cache(-25, -25, 50, 50);
	 *
	 * Note that filters need to be defined <em>before</em> the cache is applied. Check out the {{#crossLink "Filter"}}{{/crossLink}}
	 * class for more information. Some filters (ex. BlurFilter) will not work as expected in conjunction with the scale param.
	 * 
	 * Usually, the resulting cacheCanvas will have the dimensions width*scale by height*scale, however some filters (ex. BlurFilter)
	 * will add padding to the canvas dimensions.
	 *
	 * @method cache
	 * @param {Number} x The x coordinate origin for the cache region.
	 * @param {Number} y The y coordinate origin for the cache region.
	 * @param {Number} width The width of the cache region.
	 * @param {Number} height The height of the cache region.
	 * @param {Number} [scale=1] The scale at which the cache will be created. For example, if you cache a vector shape using
	 * 	myShape.cache(0,0,100,100,2) then the resulting cacheCanvas will be 200x200 px. This lets you scale and rotate
	 * 	cached elements with greater fidelity. Default is 1.
	 **/
	p.cache = function(x, y, width, height, scale) {
		// draw to canvas.
		scale = scale||1;
		if (!this.cacheCanvas) { this.cacheCanvas = createjs.createCanvas?createjs.createCanvas():document.createElement("canvas"); }
		this._cacheWidth = width;
		this._cacheHeight = height;
		this._cacheOffsetX = x;
		this._cacheOffsetY = y;
		this._cacheScale = scale;
		this.updateCache();
	};

	/**
	 * Redraws the display object to its cache. Calling updateCache without an active cache will throw an error.
	 * If compositeOperation is null the current cache will be cleared prior to drawing. Otherwise the display object
	 * will be drawn over the existing cache using the specified compositeOperation.
	 *
	 * <h4>Example</h4>
	 * Clear the current graphics of a cached shape, draw some new instructions, and then update the cache. The new line
	 * will be drawn on top of the old one.
	 *
	 *      // Not shown: Creating the shape, and caching it.
	 *      shapeInstance.clear();
	 *      shapeInstance.setStrokeStyle(3).beginStroke("#ff0000").moveTo(100, 100).lineTo(200,200);
	 *      shapeInstance.updateCache();
	 *
	 * @method updateCache
	 * @param {String} compositeOperation The compositeOperation to use, or null to clear the cache and redraw it.
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#compositing">
	 * whatwg spec on compositing</a>.
	 **/
	p.updateCache = function(compositeOperation) {
		var cacheCanvas = this.cacheCanvas, scale = this._cacheScale, offX = this._cacheOffsetX*scale, offY = this._cacheOffsetY*scale;
		var w = this._cacheWidth, h = this._cacheHeight, fBounds;
		if (!cacheCanvas) { throw "cache() must be called before updateCache()"; }
		var ctx = cacheCanvas.getContext("2d");
		
		// update bounds based on filters:
		if (fBounds = this._applyFilterBounds(offX, offY, w, h)) {
			offX = fBounds.x;
			offY = fBounds.y;
			w = fBounds.width;
			h = fBounds.height;
		}
		
		w = Math.ceil(w*scale);
		h = Math.ceil(h*scale);
		if (w != cacheCanvas.width || h != cacheCanvas.height) {
			// TODO: it would be nice to preserve the content if there is a compositeOperation.
			cacheCanvas.width = w;
			cacheCanvas.height = h;
		} else if (!compositeOperation) {
			ctx.clearRect(0, 0, w+1, h+1);
		}
		
		ctx.save();
		ctx.globalCompositeOperation = compositeOperation;
		ctx.setTransform(scale, 0, 0, scale, -offX, -offY);
		this.draw(ctx, true);
		// TODO: filters and cache scale don't play well together at present.
		this._applyFilters();
		ctx.restore();
		this.cacheID = DisplayObject._nextCacheID++;
	};

	/**
	 * Clears the current cache. See {{#crossLink "DisplayObject/cache"}}{{/crossLink}} for more information.
	 * @method uncache
	 **/
	p.uncache = function() {
		this._cacheDataURL = this.cacheCanvas = null;
		this.cacheID = this._cacheOffsetX = this._cacheOffsetY = 0;
		this._cacheScale = 1;
	};
	
	/**
	 * Returns a data URL for the cache, or null if this display object is not cached.
	 * Uses cacheID to ensure a new data URL is not generated if the cache has not changed.
	 * @method getCacheDataURL
	 * @return {String} The image data url for the cache.
	 **/
	p.getCacheDataURL = function() {
		if (!this.cacheCanvas) { return null; }
		if (this.cacheID != this._cacheDataURLID) { this._cacheDataURL = this.cacheCanvas.toDataURL(); }
		return this._cacheDataURL;
	};

	/**
	 * Returns the stage that this display object will be rendered on, or null if it has not been added to one.
	 * @method getStage
	 * @return {Stage} The Stage instance that the display object is a descendent of. null if the DisplayObject has not
	 * been added to a Stage.
	 **/
	p.getStage = function() {
		var o = this;
		while (o.parent) {
			o = o.parent;
		}
		// using dynamic access to avoid circular dependencies;
		if (o instanceof createjs["Stage"]) { return o; }
		return null;
	};

	/**
	 * Transforms the specified x and y position from the coordinate space of the display object
	 * to the global (stage) coordinate space. For example, this could be used to position an HTML label
	 * over a specific point on a nested display object. Returns a Point instance with x and y properties
	 * correlating to the transformed coordinates on the stage.
	 *
	 * <h4>Example</h4>
	 *
	 *      displayObject.x = 300;
	 *      displayObject.y = 200;
	 *      stage.addChild(displayObject);
	 *      var point = myDisplayObject.localToGlobal(100, 100);
	 *      // Results in x=400, y=300
	 *
	 * @method localToGlobal
	 * @param {Number} x The x position in the source display object to transform.
	 * @param {Number} y The y position in the source display object to transform.
	 * @return {Point} A Point instance with x and y properties correlating to the transformed coordinates
	 * on the stage.
	 **/
	p.localToGlobal = function(x, y) {
		var mtx = this.getConcatenatedMatrix(this._matrix);
		if (mtx == null) { return null; }
		mtx.append(1, 0, 0, 1, x, y);
		return new createjs.Point(mtx.tx, mtx.ty);
	};

	/**
	 * Transforms the specified x and y position from the global (stage) coordinate space to the
	 * coordinate space of the display object. For example, this could be used to determine
	 * the current mouse position within the display object. Returns a Point instance with x and y properties
	 * correlating to the transformed position in the display object's coordinate space.
	 *
	 * <h4>Example</h4>
	 *
	 *      displayObject.x = 300;
	 *      displayObject.y = 200;
	 *      stage.addChild(displayObject);
	 *      var point = myDisplayObject.globalToLocal(100, 100);
	 *      // Results in x=-200, y=-100
	 *
	 * @method globalToLocal
	 * @param {Number} x The x position on the stage to transform.
	 * @param {Number} y The y position on the stage to transform.
	 * @return {Point} A Point instance with x and y properties correlating to the transformed position in the
	 * display object's coordinate space.
	 **/
	p.globalToLocal = function(x, y) {
		var mtx = this.getConcatenatedMatrix(this._matrix);
		if (mtx == null) { return null; }
		mtx.invert();
		mtx.append(1, 0, 0, 1, x, y);
		return new createjs.Point(mtx.tx, mtx.ty);
	};

	/**
	 * Transforms the specified x and y position from the coordinate space of this display object to the coordinate
	 * space of the target display object. Returns a Point instance with x and y properties correlating to the
	 * transformed position in the target's coordinate space. Effectively the same as using the following code with
	 * {{#crossLink "DisplayObject/localToGlobal"}}{{/crossLink}} and {{#crossLink "DisplayObject/globalToLocal"}}{{/crossLink}}.
	 *
	 *      var pt = this.localToGlobal(x, y);
	 *      pt = target.globalToLocal(pt.x, pt.y);
	 *
	 * @method localToLocal
	 * @param {Number} x The x position in the source display object to transform.
	 * @param {Number} y The y position on the source display object to transform.
	 * @param {DisplayObject} target The target display object to which the coordinates will be transformed.
	 * @return {Point} Returns a Point instance with x and y properties correlating to the transformed position
	 * in the target's coordinate space.
	 **/
	p.localToLocal = function(x, y, target) {
		var pt = this.localToGlobal(x, y);
		return target.globalToLocal(pt.x, pt.y);
	};

	/**
	 * Shortcut method to quickly set the transform properties on the display object. All parameters are optional.
	 * Omitted parameters will have the default value set.
	 *
	 * <h4>Example</h4>
	 *
	 *      displayObject.setTransform(100, 100, 2, 2);
	 *
	 * @method setTransform
	 * @param {Number} [x=0] The horizontal translation (x position) in pixels
	 * @param {Number} [y=0] The vertical translation (y position) in pixels
	 * @param {Number} [scaleX=1] The horizontal scale, as a percentage of 1
	 * @param {Number} [scaleY=1] the vertical scale, as a percentage of 1
	 * @param {Number} [rotation=0] The rotation, in degrees
	 * @param {Number} [skewX=0] The horizontal skew factor
	 * @param {Number} [skewY=0] The vertical skew factor
	 * @param {Number} [regX=0] The horizontal registration point in pixels
	 * @param {Number} [regY=0] The vertical registration point in pixels
	 * @return {DisplayObject} Returns this instance. Useful for chaining commands.
	*/
	p.setTransform = function(x, y, scaleX, scaleY, rotation, skewX, skewY, regX, regY) {
		this.x = x || 0;
		this.y = y || 0;
		this.scaleX = scaleX == null ? 1 : scaleX;
		this.scaleY = scaleY == null ? 1 : scaleY;
		this.rotation = rotation || 0;
		this.skewX = skewX || 0;
		this.skewY = skewY || 0;
		this.regX = regX || 0;
		this.regY = regY || 0;
		return this;
	};
	
	/**
	 * Returns a matrix based on this object's transform.
	 * @method getMatrix
	 * @param {Matrix2D} matrix Optional. A Matrix2D object to populate with the calculated values. If null, a new
	 * Matrix object is returned.
	 * @return {Matrix2D} A matrix representing this display object's transform.
	 **/
	p.getMatrix = function(matrix) {
		var o = this;
		return (matrix ? matrix.identity() : new createjs.Matrix2D()).appendTransform(o.x, o.y, o.scaleX, o.scaleY, o.rotation, o.skewX, o.skewY, o.regX, o.regY).appendProperties(o.alpha, o.shadow, o.compositeOperation);
	};
	
	/**
	 * Generates a concatenated Matrix2D object representing the combined transform of the display object and all of its
	 * parent Containers up to the highest level ancestor (usually the {{#crossLink "Stage"}}{{/crossLink}}). This can
	 * be used to transform positions between coordinate spaces, such as with {{#crossLink "DisplayObject/localToGlobal"}}{{/crossLink}}
	 * and {{#crossLink "DisplayObject/globalToLocal"}}{{/crossLink}}.
	 * @method getConcatenatedMatrix
	 * @param {Matrix2D} [mtx] A {{#crossLink "Matrix2D"}}{{/crossLink}} object to populate with the calculated values.
	 * If null, a new Matrix2D object is returned.
	 * @return {Matrix2D} a concatenated Matrix2D object representing the combined transform of the display object and
	 * all of its parent Containers up to the highest level ancestor (usually the {{#crossLink "Stage"}}{{/crossLink}}).
	 **/
	p.getConcatenatedMatrix = function(matrix) {
		if (matrix) { matrix.identity(); }
		else { matrix = new createjs.Matrix2D(); }
		var o = this;
		while (o != null) {
			matrix.prependTransform(o.x, o.y, o.scaleX, o.scaleY, o.rotation, o.skewX, o.skewY, o.regX, o.regY).prependProperties(o.alpha, o.shadow, o.compositeOperation);
			o = o.parent;
		}
		return matrix;
	};

	/**
	 * Tests whether the display object intersects the specified local point (ie. draws a pixel with alpha > 0 at
	 * the specified position). This ignores the alpha, shadow and compositeOperation of the display object, and all
	 * transform properties including regX/Y.
	 *
	 * <h4>Example</h4>
	 *
	 *      stage.addEventListener("stagemousedown", handleMouseDown);
	 *      function handleMouseDown(event) {
	 *          var hit = myShape.hitTest(event.stageX, event.stageY);
	 *      }
	 *
	 * Please note that shape-to-shape collision is not currently supported by EaselJS.
	 * @method hitTest
	 * @param {Number} x The x position to check in the display object's local coordinates.
	 * @param {Number} y The y position to check in the display object's local coordinates.
	 * @return {Boolean} A Boolean indicting whether a visible portion of the DisplayObject intersect the specified
	 * local Point.
	*/
	p.hitTest = function(x, y) {
		// TODO: update with support for .hitArea and update hitArea docs?
		var ctx = DisplayObject._hitTestContext;
		ctx.setTransform(1, 0, 0, 1, -x, -y);
		this.draw(ctx);

		var hit = this._testHit(ctx);
		ctx.setTransform(1, 0, 0, 1, 0, 0);
		ctx.clearRect(0, 0, 2, 2);
		return hit;
	};
	
	/**
	 * Provides a chainable shortcut method for setting a number of properties on a DisplayObject instance.
	 *
	 * <h4>Example</h4>
	 *
	 *      var myGraphics = new createjs.Graphics().beginFill("#ff0000").drawCircle(0, 0, 25);
	 *      var shape = stage.addChild(new Shape())
	 *          .set({graphics:myGraphics, x:100, y:100, alpha:0.5});
	 *
	 * @method set
	 * @param {Object} props A generic object containing properties to copy to the DisplayObject instance.
	 * @return {DisplayObject} Returns The DisplayObject instance the method is called on (useful for chaining calls.)
	*/
	p.set = function(props) {
		for (var n in props) { this[n] = props[n]; }
		return this;
	};
	
	/**
	 * Returns a rectangle representing this object's bounds in its local coordinate system (ie. with no transformation).
	 * Objects that have been cached will return the bounds of the cache.
	 * 
	 * Not all display objects can calculate their own bounds (ex. Shape). For these objects, you can use 
	 * {{#crossLink "DisplayObject/setBounds"}}{{/crossLink}} so that they are included when calculating Container
	 * bounds.
	 * 
	 * <table>
	 * 	<tr><td><b>All</b></td><td>
	 * 		All display objects support setting bounds manually using setBounds(). Likewise, display objects that
	 * 		have been cached using cache() will return the bounds of their cache. Manual and cache bounds will override
	 * 		the automatic calculations listed below.
	 * 	</td></tr>
	 * 	<tr><td><b>Bitmap</b></td><td>
	 * 		Returns the width and height of the sourceRect (if specified) or image, extending from (x=0,y=0).
	 * 	</td></tr>
	 * 	<tr><td><b>Sprite</b></td><td>
	 * 		Returns the bounds of the current frame. May have non-zero x/y if a frame registration point was specified
	 * 		in the spritesheet data. See also {{#crossLink "SpriteSheet/getFrameBounds"}}{{/crossLink}}
	 * 	</td></tr>
	 * 	<tr><td><b>Container</b></td><td>
	 * 		Returns the aggregate (combined) bounds of all children that return a non-null value from getBounds().
	 * 	</td></tr>
	 * 	<tr><td><b>Shape</b></td><td>
	 * 		Does not currently support automatic bounds calculations. Use setBounds() to manually define bounds.
	 * 	</td></tr>
	 * 	<tr><td><b>Text</b></td><td>
	 * 		Returns approximate bounds. Horizontal values (x/width) are quite accurate, but vertical values (y/height) are
	 * 		not, especially when using textBaseline values other than "top".
	 * 	</td></tr>
	 * 	<tr><td><b>BitmapText</b></td><td>
	 * 		Returns approximate bounds. Values will be more accurate if spritesheet frame registration points are close
	 * 		to (x=0,y=0).
	 * 	</td></tr>
	* </table>
	 * 
	 * Bounds can be expensive to calculate for some objects (ex. text, or containers with many children), and
	 * are recalculated each time you call getBounds(). You can prevent recalculation on static objects by setting the
	 * bounds explicitly:
	 * 
	 * 	var bounds = obj.getBounds();
	 * 	obj.setBounds(bounds.x, bounds.y, bounds.width, bounds.height);
	 * 	// getBounds will now use the set values, instead of recalculating
	 * 
	 * To reduce memory impact, the returned Rectangle instance may be reused internally; clone the instance or copy its
	 * values if you need to retain it.
	 * 
	 * 	var myBounds = obj.getBounds().clone();
	 * 	// OR:
	 * 	myRect.copy(obj.getBounds());
	 * 
	 * @method getBounds
	 * @return {Rectangle} A Rectangle instance representing the bounds, or null if bounds are not available for this
	 * object.
	 **/
	p.getBounds = function() {
		if (this._bounds) { return this._rectangle.copy(this._bounds); }
		var cacheCanvas = this.cacheCanvas;
		if (cacheCanvas) {
			var scale = this._cacheScale;
			return this._rectangle.initialize(this._cacheOffsetX, this._cacheOffsetY, cacheCanvas.width/scale, cacheCanvas.height/scale);
		}
		return null;
	};
	
	/**
	 * Returns a rectangle representing this object's bounds in its parent's coordinate system (ie. with transformations applied).
	 * Objects that have been cached will return the transformed bounds of the cache.
	 * 
	 * Not all display objects can calculate their own bounds (ex. Shape). For these objects, you can use 
	 * {{#crossLink "DisplayObject/setBounds"}}{{/crossLink}} so that they are included when calculating Container
	 * bounds.
	 * 
	 * To reduce memory impact, the returned Rectangle instance may be reused internally; clone the instance or copy its
	 * values if you need to retain it.
	 * 
	 * Container instances calculate aggregate bounds for all children that return bounds via getBounds.
	 * @method getTransformedBounds
	 * @return {Rectangle} A Rectangle instance representing the bounds, or null if bounds are not available for this object.
	 **/
	p.getTransformedBounds = function() {
		return this._getBounds();
	};
	
	/**
	 * Allows you to manually specify the bounds of an object that either cannot calculate their own bounds (ex. Shape &
	 * Text) for future reference, or so the object can be included in Container bounds. Manually set bounds will always
	 * override calculated bounds.
	 * 
	 * The bounds should be specified in the object's local (untransformed) coordinates. For example, a Shape instance
	 * with a 25px radius circle centered at 0,0 would have bounds of (-25, -25, 50, 50).
	 * @method setBounds
	 * @param {Number} x The x origin of the bounds. Pass null to remove the manual bounds.
	 * @param {Number} y The y origin of the bounds.
	 * @param {Number} width The width of the bounds.
	 * @param {Number} height The height of the bounds.
	 **/
	p.setBounds = function(x, y, width, height) {
		if (x == null) { this._bounds = x; }
		this._bounds = (this._bounds || new createjs.Rectangle()).initialize(x, y, width, height);
	};

	/**
	 * Returns a clone of this DisplayObject. Some properties that are specific to this instance's current context are
	 * reverted to their defaults (for example .parent). Also note that caches are not maintained across clones.
	 * @method clone
	 * @return {DisplayObject} A clone of the current DisplayObject instance.
	 **/
	p.clone = function() {
		var o = new DisplayObject();
		this.cloneProps(o);
		return o;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[DisplayObject (name="+  this.name +")]";
	};

// private methods:

	// separated so it can be used more easily in subclasses:
	/**
	 * @method cloneProps
	 * @protected
	 * @param {DisplayObject} o The DisplayObject instance which will have properties from the current DisplayObject
	 * instance copied into.
	 **/
	p.cloneProps = function(o) {
		o.alpha = this.alpha;
		o.name = this.name;
		o.regX = this.regX;
		o.regY = this.regY;
		o.rotation = this.rotation;
		o.scaleX = this.scaleX;
		o.scaleY = this.scaleY;
		o.shadow = this.shadow;
		o.skewX = this.skewX;
		o.skewY = this.skewY;
		o.visible = this.visible;
		o.x  = this.x;
		o.y = this.y;
		o._bounds = this._bounds;
		o.mouseEnabled = this.mouseEnabled;
		o.compositeOperation = this.compositeOperation;
	};

	/**
	 * @method _applyShadow
	 * @protected
	 * @param {CanvasRenderingContext2D} ctx
	 * @param {Shadow} shadow
	 **/
	p._applyShadow = function(ctx, shadow) {
		shadow = shadow || Shadow.identity;
		ctx.shadowColor = shadow.color;
		ctx.shadowOffsetX = shadow.offsetX;
		ctx.shadowOffsetY = shadow.offsetY;
		ctx.shadowBlur = shadow.blur;
	};
	
	
	/**
	 * @method _tick
	 * @param {Array} params Parameters to pass on to any listeners of the tick function. This will usually include the
	 * properties from the {{#crossLink "Ticker"}}{{/crossLink}} "tick" event, such as `delta` and `paused`, but may
	 * be undefined or contain other values depending on the usage by the application.
	 * @protected
	 **/
	p._tick = function(params) {
		// because tick can be really performance sensitive, we'll inline some of the dispatchEvent work.
		var ls = this._listeners;
		if (ls && ls["tick"]) {
			var evt = new createjs.Event("tick");
			evt.params = params;
			this._dispatchEvent(evt, this, 2);
		}
	};

	/**
	 * @method _testHit
	 * @protected
	 * @param {CanvasRenderingContext2D} ctx
	 * @return {Boolean}
	 **/
	p._testHit = function(ctx) {
		try {
			var hit = ctx.getImageData(0, 0, 1, 1).data[3] > 1;
		} catch (e) {
			if (!DisplayObject.suppressCrossDomainErrors) {
				throw "An error has occurred. This is most likely due to security restrictions on reading canvas pixel data with local or cross-domain images.";
			}
		}
		return hit;
	};

	/**
	 * @method _applyFilters
	 * @protected
	 **/
	p._applyFilters = function() {
		if (!this.filters || this.filters.length == 0 || !this.cacheCanvas) { return; }
		var l = this.filters.length;
		var ctx = this.cacheCanvas.getContext("2d");
		var w = this.cacheCanvas.width;
		var h = this.cacheCanvas.height;
		for (var i=0; i<l; i++) {
			this.filters[i].applyFilter(ctx, 0, 0, w, h);
		}
	};
	
	/**
	 * @method _applyFilterBounds
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Number} width
	 * @param {Number} height
	 * @return {Rectangle}
	 * @protected
	 **/
	p._applyFilterBounds = function(x, y, width, height) {
		var bounds, l, filters = this.filters;
		if (!filters || !(l=filters.length)) { return null; }
		
		for (var i=0; i<l; i++) {
			var f = this.filters[i];
			var fBounds = f.getBounds&&f.getBounds();
			if (!fBounds) { continue; }
			if (!bounds) { bounds = this._rectangle.initialize(x,y,width,height); }
			bounds.x += fBounds.x;
			bounds.y += fBounds.y;
			bounds.width += fBounds.width;
			bounds.height += fBounds.height;
		}
		return bounds;
	};
	
	/**
	 * @method _getBounds
	 * @param {Matrix2D} matrix
	 * @param {Boolean} ignoreTransform If true, does not apply this object's transform.
	 * @return {Rectangle}
	 * @protected
	 **/
	p._getBounds = function(matrix, ignoreTransform){
		return this._transformBounds(this.getBounds(), matrix, ignoreTransform);
	};
	
	/**
	 * @method _transformBounds
	 * @param {Rectangle} bounds
	 * @param {Matrix2D} matrix
	 * @param {Boolean} ignoreTransform
	 * @return {Rectangle}
	 * @protected
	 **/
	p._transformBounds = function(bounds, matrix, ignoreTransform) {
		if (!bounds) { return bounds; }
		var x = bounds.x, y = bounds.y, width = bounds.width, height = bounds.height;
		var mtx = ignoreTransform ? this._matrix.identity() : this.getMatrix(this._matrix);
		
		if (x || y) { mtx.appendTransform(0,0,1,1,0,0,0,-x,-y); }
		if (matrix) { mtx.prependMatrix(matrix); }
		
		var x_a = width*mtx.a, x_b = width*mtx.b;
		var y_c = height*mtx.c, y_d = height*mtx.d;
		var tx = mtx.tx, ty = mtx.ty;
		
		var minX = tx, maxX = tx, minY = ty, maxY = ty;

		if ((x = x_a + tx) < minX) { minX = x; } else if (x > maxX) { maxX = x; }
		if ((x = x_a + y_c + tx) < minX) { minX = x; } else if (x > maxX) { maxX = x; }
		if ((x = y_c + tx) < minX) { minX = x; } else if (x > maxX) { maxX = x; }
		
		if ((y = x_b + ty) < minY) { minY = y; } else if (y > maxY) { maxY = y; }
		if ((y = x_b + y_d + ty) < minY) { minY = y; } else if (y > maxY) { maxY = y; }
		if ((y = y_d + ty) < minY) { minY = y; } else if (y > maxY) { maxY = y; }
		
		return bounds.initialize(minX, minY, maxX-minX, maxY-minY);
	};
	
	/**
	 * Indicates whether the display object has any mouse event listeners or a cursor.
	 * @method _isMouseOpaque
	 * @return {Boolean}
	 * @protected
	 **/
	p._hasMouseEventListener = function() {
		var evts = DisplayObject._MOUSE_EVENTS;
		for (var i= 0, l=evts.length; i<l; i++) {
			if (this.hasEventListener(evts[i])) { return true; }
		}
		return !!this.cursor;
	};

createjs.DisplayObject = DisplayObject;
}());/*
* Container
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
* 
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
* 
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

// namespace:
this.createjs = this.createjs||{};

(function() {

/**
 * A Container is a nestable display list that allows you to work with compound display elements. For  example you could
 * group arm, leg, torso and head {{#crossLink "Bitmap"}}{{/crossLink}} instances together into a Person Container, and
 * transform them as a group, while still being able to move the individual parts relative to each other. Children of
 * containers have their <code>transform</code> and <code>alpha</code> properties concatenated with their parent
 * Container.
 *
 * For example, a {{#crossLink "Shape"}}{{/crossLink}} with x=100 and alpha=0.5, placed in a Container with <code>x=50</code>
 * and <code>alpha=0.7</code> will be rendered to the canvas at <code>x=150</code> and <code>alpha=0.35</code>.
 * Containers have some overhead, so you generally shouldn't create a Container to hold a single child.
 *
 * <h4>Example</h4>
 *      var container = new createjs.Container();
 *      container.addChild(bitmapInstance, shapeInstance);
 *      container.x = 100;
 *
 * @class Container
 * @extends DisplayObject
 * @constructor
 **/
var Container = function() {
  this.initialize();
};
var p = Container.prototype = new createjs.DisplayObject();

// public properties:
	/**
	 * The array of children in the display list. You should usually use the child management methods such as
	 * {{#crossLink "Container/addChild"}}{{/crossLink}}, {{#crossLink "Container/removeChild"}}{{/crossLink}},
	 * {{#crossLink "Container/swapChildren"}}{{/crossLink}}, etc, rather than accessing this directly, but it is
	 * included for advanced uses.
	 * @property children
	 * @type Array
	 * @default null
	 **/
	p.children = null;
	
	/**
	 * Indicates whether the children of this container are independently enabled for mouse/pointer interaction.
	 * If false, the children will be aggregated under the container - for example, a click on a child shape would
	 * trigger a click event on the container.
	 * @property mouseChildren
	 * @type Boolean
	 * @default true
	 **/
	p.mouseChildren = true;
	
	/**
	 * If false, the tick will not be propagated to children of this Container. This can provide some performance benefits.
	 * In addition to preventing the "tick" event from being dispatched, it will also prevent tick related updates
	 * on some display objects (ex. Sprite & MovieClip frame advancing, DOMElement visibility handling).
	 * @property tickChildren
	 * @type Boolean
	 * @default true
	 **/
	p.tickChildren = true;

// constructor:

	/**
	 * @property DisplayObject_initialize
	 * @type Function
	 * @private
	 **/
	p.DisplayObject_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	*/
	p.initialize = function() {
		this.DisplayObject_initialize();
		this.children = [];
	};

// public methods:

	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the display object would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		var hasContent = this.cacheCanvas || this.children.length;
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0 && hasContent);
	};

	/**
	 * @property DisplayObject_draw
	 * @type Function
	 * @private
	 **/
	p.DisplayObject_draw = p.draw;

	/**
	 * Draws the display object into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} [ignoreCache=false] Indicates whether the draw operation should ignore any current cache.
	 * For example, used for drawing the cache (to prevent it from simply drawing an existing cache back
	 * into itself).
	 **/
	p.draw = function(ctx, ignoreCache) {
		if (this.DisplayObject_draw(ctx, ignoreCache)) { return true; }
		
		// this ensures we don't have issues with display list changes that occur during a draw:
		var list = this.children.slice(0);
		for (var i=0,l=list.length; i<l; i++) {
			var child = list[i];
			if (!child.isVisible()) { continue; }
			
			// draw the child:
			ctx.save();
			child.updateContext(ctx);
			child.draw(ctx);
			ctx.restore();
		}
		return true;
	};
	
	/**
	 * Adds a child to the top of the display list.
	 *
	 * <h4>Example</h4>
	 *      container.addChild(bitmapInstance);
	 *
	 *  You can also add multiple children at once:
	 *
	 *      container.addChild(bitmapInstance, shapeInstance, textInstance);
	 *
	 * @method addChild
	 * @param {DisplayObject} child The display object to add.
	 * @return {DisplayObject} The child that was added, or the last child if multiple children were added.
	 **/
	p.addChild = function(child) {
		if (child == null) { return child; }
		var l = arguments.length;
		if (l > 1) {
			for (var i=0; i<l; i++) { this.addChild(arguments[i]); }
			return arguments[l-1];
		}
		if (child.parent) { child.parent.removeChild(child); }
		child.parent = this;
		this.children.push(child);
		return child;
	};

	/**
	 * Adds a child to the display list at the specified index, bumping children at equal or greater indexes up one, and
	 * setting its parent to this Container.
	 *
	 * <h4>Example</h4>
	 *      addChildAt(child1, index);
	 *
	 * You can also add multiple children, such as:
	 *
	 *      addChildAt(child1, child2, ..., index);
	 *
	 * The index must be between 0 and numChildren. For example, to add myShape under otherShape in the display list,
	 * you could use:
	 *
	 *      container.addChildAt(myShape, container.getChildIndex(otherShape));
	 *
	 * This would also bump otherShape's index up by one. Fails silently if the index is out of range.
	 *
	 * @method addChildAt
	 * @param {DisplayObject} child The display object to add.
	 * @param {Number} index The index to add the child at.
	 * @return {DisplayObject} Returns the last child that was added, or the last child if multiple children were added.
	 **/
	p.addChildAt = function(child, index) {
		var l = arguments.length;
		var indx = arguments[l-1]; // can't use the same name as the index param or it replaces arguments[1]
		if (indx < 0 || indx > this.children.length) { return arguments[l-2]; }
		if (l > 2) {
			for (var i=0; i<l-1; i++) { this.addChildAt(arguments[i], indx+i); }
			return arguments[l-2];
		}
		if (child.parent) { child.parent.removeChild(child); }
		child.parent = this;
		this.children.splice(index, 0, child);
		return child;
	};

	/**
	 * Removes the specified child from the display list. Note that it is faster to use removeChildAt() if the index is
	 * already known.
	 *
	 * <h4>Example</h4>
	 *      container.removeChild(child);
	 *
	 * You can also remove multiple children:
	 *
	 *      removeChild(child1, child2, ...);
	 *
	 * Returns true if the child (or children) was removed, or false if it was not in the display list.
	 * @method removeChild
	 * @param {DisplayObject} child The child to remove.
	 * @return {Boolean} true if the child (or children) was removed, or false if it was not in the display list.
	 **/
	p.removeChild = function(child) {
		var l = arguments.length;
		if (l > 1) {
			var good = true;
			for (var i=0; i<l; i++) { good = good && this.removeChild(arguments[i]); }
			return good;
		}
		return this.removeChildAt(createjs.indexOf(this.children, child));
	};

	/**
	 * Removes the child at the specified index from the display list, and sets its parent to null.
	 *
	 * <h4>Example</h4>
	 *
	 *      container.removeChildAt(2);
	 *
	 * You can also remove multiple children:
	 *
	 *      container.removeChild(2, 7, ...)
	 *
	 * Returns true if the child (or children) was removed, or false if any index was out of range.
	 * @method removeChildAt
	 * @param {Number} index The index of the child to remove.
	 * @return {Boolean} true if the child (or children) was removed, or false if any index was out of range.
	 **/
	p.removeChildAt = function(index) {
		var l = arguments.length;
		if (l > 1) {
			var a = [];
			for (var i=0; i<l; i++) { a[i] = arguments[i]; }
			a.sort(function(a, b) { return b-a; });
			var good = true;
			for (var i=0; i<l; i++) { good = good && this.removeChildAt(a[i]); }
			return good;
		}
		if (index < 0 || index > this.children.length-1) { return false; }
		var child = this.children[index];
		if (child) { child.parent = null; }
		this.children.splice(index, 1);
		return true;
	};

	/**
	 * Removes all children from the display list.
	 *
	 * <h4>Example</h4>
	 *      container.removeAlLChildren();
	 *
	 * @method removeAllChildren
	 **/
	p.removeAllChildren = function() {
		var kids = this.children;
		while (kids.length) { kids.pop().parent = null; }
	};

	/**
	 * Returns the child at the specified index.
	 *
	 * <h4>Example</h4>
	 *      container.getChildAt(2);
	 *
	 * @method getChildAt
	 * @param {Number} index The index of the child to return.
	 * @return {DisplayObject} The child at the specified index. Returns null if there is no child at the index.
	 **/
	p.getChildAt = function(index) {
		return this.children[index];
	};
	
	/**
	 * Returns the child with the specified name.
	 * @method getChildByName
	 * @param {String} name The name of the child to return.
	 * @return {DisplayObject} The child with the specified name.
	 **/
	p.getChildByName = function(name) {
		var kids = this.children;
		for (var i=0,l=kids.length;i<l;i++) {
			if(kids[i].name == name) { return kids[i]; }
		}
		return null;
	};

	/**
	 * Performs an array sort operation on the child list.
	 *
	 * <h4>Example: Display children with a higher y in front.</h4>
	 * 
	 *      var sortFunction = function(obj1, obj2, options) {
	 *          if (obj1.y > obj2.y) { return 1; }
	 *          if (obj1.y < obj2.y) { return -1; }
	 *          return 0;
	 *      }
	 *      container.sortChildren(sortFunction);
	 *
	 * @method sortChildren
	 * @param {Function} sortFunction the function to use to sort the child list. See JavaScript's <code>Array.sort</code>
	 * documentation for details.
	 **/
	p.sortChildren = function(sortFunction) {
		this.children.sort(sortFunction);
	};

	/**
	 * Returns the index of the specified child in the display list, or -1 if it is not in the display list.
	 *
	 * <h4>Example</h4>
	 *      var index = container.getChildIndex(child);
	 *
	 * @method getChildIndex
	 * @param {DisplayObject} child The child to return the index of.
	 * @return {Number} The index of the specified child. -1 if the child is not found.
	 **/
	p.getChildIndex = function(child) {
		return createjs.indexOf(this.children, child);
	};

	/**
	 * Returns the number of children in the display list.
	 * @method getNumChildren
	 * @return {Number} The number of children in the display list.
	 **/
	p.getNumChildren = function() {
		return this.children.length;
	};
	
	/**
	 * Swaps the children at the specified indexes. Fails silently if either index is out of range.
	 * @method swapChildrenAt
	 * @param {Number} index1
	 * @param {Number} index2
	 **/
	p.swapChildrenAt = function(index1, index2) {
		var kids = this.children;
		var o1 = kids[index1];
		var o2 = kids[index2];
		if (!o1 || !o2) { return; }
		kids[index1] = o2;
		kids[index2] = o1;
	};
	
	/**
	 * Swaps the specified children's depth in the display list. Fails silently if either child is not a child of this
	 * Container.
	 * @method swapChildren
	 * @param {DisplayObject} child1
	 * @param {DisplayObject} child2
	 **/
	p.swapChildren = function(child1, child2) {
		var kids = this.children;
		var index1,index2;
		for (var i=0,l=kids.length;i<l;i++) {
			if (kids[i] == child1) { index1 = i; }
			if (kids[i] == child2) { index2 = i; }
			if (index1 != null && index2 != null) { break; }
		}
		if (i==l) { return; } // TODO: throw error?
		kids[index1] = child2;
		kids[index2] = child1;
	};
	
	/**
	 * Changes the depth of the specified child. Fails silently if the child is not a child of this container, or the index is out of range.
	 * @param {DisplayObject} child
	 * @param {Number} index  
	 * @method setChildIndex
	 **/
	p.setChildIndex = function(child, index) {
		var kids = this.children, l=kids.length;
		if (child.parent != this || index < 0 || index >= l) { return; }
		for (var i=0;i<l;i++) {
			if (kids[i] == child) { break; }
		}
		if (i==l || i == index) { return; }
		kids.splice(i,1);
		kids.splice(index,0,child);
	};

	/**
	 * Returns true if the specified display object either is this container or is a descendent (child, grandchild, etc)
	 * of this container.
	 * @method contains
	 * @param {DisplayObject} child The DisplayObject to be checked.
	 * @return {Boolean} true if the specified display object either is this container or is a descendent.
	 **/
	p.contains = function(child) {
		while (child) {
			if (child == this) { return true; }
			child = child.parent;
		}
		return false;
	};

	/**
	 * Tests whether the display object intersects the specified local point (ie. draws a pixel with alpha > 0 at the
	 * specified position). This ignores the alpha, shadow and compositeOperation of the display object, and all
	 * transform properties including regX/Y.
	 * @method hitTest
	 * @param {Number} x The x position to check in the display object's local coordinates.
	 * @param {Number} y The y position to check in the display object's local coordinates.
	 * @return {Boolean} A Boolean indicating whether there is a visible section of a DisplayObject that overlaps the specified
	 * coordinates.
	 **/
	p.hitTest = function(x, y) {
		// TODO: optimize to use the fast cache check where possible.
		return (this.getObjectUnderPoint(x, y) != null);
	};

	/**
	 * Returns an array of all display objects under the specified coordinates that are in this container's display
	 * list. This routine ignores any display objects with mouseEnabled set to false. The array will be sorted in order
	 * of visual depth, with the top-most display object at index 0. This uses shape based hit detection, and can be an
	 * expensive operation to run, so it is best to use it carefully. For example, if testing for objects under the
	 * mouse, test on tick (instead of on mousemove), and only if the mouse's position has changed.
	 * @method getObjectsUnderPoint
	 * @param {Number} x The x position in the container to test.
	 * @param {Number} y The y position in the container to test.
	 * @return {Array} An Array of DisplayObjects under the specified coordinates.
	 **/
	p.getObjectsUnderPoint = function(x, y) {
		var arr = [];
		var pt = this.localToGlobal(x, y);
		this._getObjectsUnderPoint(pt.x, pt.y, arr);
		return arr;
	};

	/**
	 * Similar to {{#crossLink "Container/getObjectsUnderPoint()"}}{{/crossLink}}, but returns only the top-most display
	 * object. This runs significantly faster than <code>getObjectsUnderPoint()<code>, but is still an expensive
	 * operation. See {{#crossLink "Container/getObjectsUnderPoint"}}{{/crossLink}} for more information.
	 * @method getObjectUnderPoint
	 * @param {Number} x The x position in the container to test.
	 * @param {Number} y The y position in the container to test.
	 * @return {DisplayObject} The top-most display object under the specified coordinates.
	 **/
	p.getObjectUnderPoint = function(x, y) {
		var pt = this.localToGlobal(x, y);
		return this._getObjectsUnderPoint(pt.x, pt.y);
	};
	
	/**
	 * @property DisplayObject_getBounds
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_getBounds = p.getBounds; 
	
	/**
	 * Docced in superclass.
	 */
	p.getBounds = function() {
		return this._getBounds(null, true);
	};
	
	
	/**
	 * Docced in superclass.
	 */
	p.getTransformedBounds = function() {
		return this._getBounds();
	};

	/**
	 * Returns a clone of this Container. Some properties that are specific to this instance's current context are
	 * reverted to their defaults (for example .parent).
	 * @method clone
	 * @param {Boolean} recursive If true, all of the descendants of this container will be cloned recursively. If false, the
	 * properties of the container will be cloned, but the new instance will not have any children.
	 * @return {Container} A clone of the current Container instance.
	 **/
	p.clone = function(recursive) {
		var o = new Container();
		this.cloneProps(o);
		if (recursive) {
			var arr = o.children = [];
			for (var i=0, l=this.children.length; i<l; i++) {
				var clone = this.children[i].clone(recursive);
				clone.parent = o;
				arr.push(clone);
			}
		}
		return o;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Container (name="+  this.name +")]";
	};

// private properties:
	/**
	 * @property DisplayObject__tick
	 * @type Function
	 * @private
	 **/
	p.DisplayObject__tick = p._tick;
	
	/**
	 * @method _tick
	 * @param {Array} params Parameters to pass onto the DisplayObject {{#crossLink "DisplayObject/tick"}}{{/crossLink}}
	 * function.
	 * @protected
	 **/
	p._tick = function(params) {
		if (this.tickChildren) {
			for (var i=this.children.length-1; i>=0; i--) {
				var child = this.children[i];
				if (child.tickEnabled && child._tick) { child._tick(params); }
			}
		}
		this.DisplayObject__tick(params);
	};

	/**
	 * @method _getObjectsUnderPoint
	 * @param {Number} x
	 * @param {Number} y
	 * @param {Array} arr
	 * @param {Boolean} mouse If true, it will respect mouse interaction properties like mouseEnabled, mouseChildren, and active listeners.
	 * @param {Boolean} activeListener If true, there is an active mouse event listener.
	 * @return {Array}
	 * @protected
	 **/
	p._getObjectsUnderPoint = function(x, y, arr, mouse, activeListener) {
		var ctx = createjs.DisplayObject._hitTestContext;
		var mtx = this._matrix;
		activeListener = activeListener || (mouse&&this._hasMouseEventListener());

		// draw children one at a time, and check if we get a hit:
		var children = this.children;
		var l = children.length;
		for (var i=l-1; i>=0; i--) {
			var child = children[i];
			var hitArea = child.hitArea;
			if (!child.visible || (!hitArea && !child.isVisible()) || (mouse && !child.mouseEnabled)) { continue; }
			// if a child container has a hitArea then we only need to check its hitArea, so we can treat it as a normal DO:
			if (!hitArea && child instanceof Container) {
				var result = child._getObjectsUnderPoint(x, y, arr, mouse, activeListener);
				if (!arr && result) { return (mouse && !this.mouseChildren) ? this : result; }
			} else {
				if (!activeListener && !child._hasMouseEventListener()) { continue; }
				
				child.getConcatenatedMatrix(mtx);
				
				if (hitArea) {
					mtx.appendTransform(hitArea.x, hitArea.y, hitArea.scaleX, hitArea.scaleY, hitArea.rotation, hitArea.skewX, hitArea.skewY, hitArea.regX, hitArea.regY);
					mtx.alpha = hitArea.alpha;
				}
				
				ctx.globalAlpha = mtx.alpha;
				ctx.setTransform(mtx.a,  mtx.b, mtx.c, mtx.d, mtx.tx-x, mtx.ty-y);
				(hitArea||child).draw(ctx);
				if (!this._testHit(ctx)) { continue; }
				ctx.setTransform(1, 0, 0, 1, 0, 0);
				ctx.clearRect(0, 0, 2, 2);
				if (arr) { arr.push(child); }
				else { return (mouse && !this.mouseChildren) ? this : child; }
			}
		}
		return null;
	};
	
	/**
	 * @method _getBounds
	 * @param {Matrix2D} matrix
	 * @param {Boolean} ignoreTransform If true, does not apply this object's transform.
	 * @return {Rectangle}
	 * @protected
	 **/
	p._getBounds = function(matrix, ignoreTransform) {
		var bounds = this.DisplayObject_getBounds();
		if (bounds) { return this._transformBounds(bounds, matrix, ignoreTransform); }
		
		var minX, maxX, minY, maxY;
		var mtx = ignoreTransform ? this._matrix.identity() : this.getMatrix(this._matrix);
		if (matrix) { mtx.prependMatrix(matrix); }
		
		var l = this.children.length;
		for (var i=0; i<l; i++) {
			var child = this.children[i];
			if (!child.visible || !(bounds = child._getBounds(mtx))) { continue; }
			var x1=bounds.x, y1=bounds.y, x2=x1+bounds.width, y2=y1+bounds.height;
			if (x1 < minX || minX == null) { minX = x1; }
			if (x2 > maxX || maxX == null) { maxX = x2; }
			if (y1 < minY || minY == null) { minY = y1; }
			if (y2 > maxY || maxY == null) { maxY = y2; }
		}
		
		return (maxX == null) ? null : this._rectangle.initialize(minX, minY, maxX-minX, maxY-minY);
	};

createjs.Container = Container;
}());/*
* Stage
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * A stage is the root level {{#crossLink "Container"}}{{/crossLink}} for a display list. Each time its {{#crossLink "Stage/tick"}}{{/crossLink}}
 * method is called, it will render its display list to its target canvas.
 *
 * <h4>Example</h4>
 * This example creates a stage, adds a child to it, then uses {{#crossLink "Ticker"}}{{/crossLink}} to update the child
 * and redraw the stage using {{#crossLink "Stage/update"}}{{/crossLink}}.
 *
 *      var stage = new createjs.Stage("canvasElementId");
 *      var image = new createjs.Bitmap("imagePath.png");
 *      stage.addChild(image);
 *      createjs.Ticker.addEventListener("tick", handleTick);
 *      function handleTick(event) {
 *          image.x += 10;
 *          stage.update();
 *      }
 *
 * @class Stage
 * @extends Container
 * @constructor
 * @param {HTMLCanvasElement | String | Object} canvas A canvas object that the Stage will render to, or the string id
 * of a canvas object in the current document.
 **/
var Stage = function(canvas) {
  this.initialize(canvas);
};
var p = Stage.prototype = new createjs.Container();

// static properties:
	/**
	 * @property _snapToPixelEnabled
	 * @protected
	 * @static
	 * @type {Boolean}
	 * @default false
	 * @deprecated Hardware acceleration in modern browsers makes this unnecessary.
	 **/
	Stage._snapToPixelEnabled = false; // snapToPixelEnabled is temporarily copied here during a draw to provide global access.

// events:

	/**
	 * Dispatched when the user moves the mouse over the canvas.
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event stagemousemove
	 * @since 0.6.0
	 */

	/**
	 * Dispatched when the user presses their left mouse button on the canvas. See the {{#crossLink "MouseEvent"}}{{/crossLink}}
	 * class for a listing of event properties.
	 * @event stagemousedown
	 * @since 0.6.0
	 */

	/**
	 * Dispatched when the user the user releases the mouse button anywhere that the page can detect it (this varies slightly between browsers).
	 * See the {{#crossLink "MouseEvent"}}{{/crossLink}} class for a listing of event properties.
	 * @event stagemouseup
	 * @since 0.6.0
	 */

	/**
	 * Dispatched when the mouse moves from within the canvas area (mouseInBounds == true) to outside it (mouseInBounds == false).
	 * This is currently only dispatched for mouse input (not touch). See the {{#crossLink "MouseEvent"}}{{/crossLink}}
	 * class for a listing of event properties.
	 * @event mouseleave
	 * @since 0.7.0
	 */

	/**
	 * Dispatched when the mouse moves into the canvas area (mouseInBounds == false) from outside it (mouseInBounds == true).
	 * This is currently only dispatched for mouse input (not touch). See the {{#crossLink "MouseEvent"}}{{/crossLink}}
	 * class for a listing of event properties.
	 * @event mouseenter
	 * @since 0.7.0
	 */
	 
	/**
	 * Dispatched each update immediately before the tick event is propagated through the display list. Does not fire if
	 * tickOnUpdate is false.
	 * @event tickstart
	 * @since 0.7.0
	 */
	 
	/**
	 * Dispatched each update immediately after the tick event is propagated through the display list. Does not fire if
	 * tickOnUpdate is false. Precedes the "drawstart" event.
	 * @event tickend
	 * @since 0.7.0
	 */
	 
	/**
	 * Dispatched each update immediately before the canvas is cleared and the display list is drawn to it.
	 * @event drawstart
	 * @since 0.7.0
	 */
	 
	/**
	 * Dispatched each update immediately after the display list is drawn to the canvas and the canvas context is restored.
	 * @event drawend
	 * @since 0.7.0
	 */

// public properties:
	/**
	 * Indicates whether the stage should automatically clear the canvas before each render. You can set this to <code>false</code>
	 * to manually control clearing (for generative art, or when pointing multiple stages at the same canvas for
	 * example).
	 *
	 * <h4>Example</h4>
	 *
	 *      var stage = new createjs.Stage("canvasId");
	 *      stage.autoClear = false;
	 *
	 * @property autoClear
	 * @type Boolean
	 * @default true
	 **/
	p.autoClear = true;

	/**
	 * The canvas the stage will render to. Multiple stages can share a single canvas, but you must disable autoClear for all but the
	 * first stage that will be ticked (or they will clear each other's render).
	 *
	 * When changing the canvas property you must disable the events on the old canvas, and enable events on the
	 * new canvas or mouse events will not work as expected. For example:
	 *
	 *      myStage.enableDOMEvents(false);
	 *      myStage.canvas = anotherCanvas;
	 *      myStage.enableDOMEvents(true);
	 *
	 * @property canvas
	 * @type HTMLCanvasElement | Object
	 **/
	p.canvas = null;

	/**
	 * The current mouse X position on the canvas. If the mouse leaves the canvas, this will indicate the most recent
	 * position over the canvas, and mouseInBounds will be set to false.
	 * @property mouseX
	 * @type Number
	 * @readonly
	 **/
	p.mouseX = 0;

	/**
	 * The current mouse Y position on the canvas. If the mouse leaves the canvas, this will indicate the most recent
	 * position over the canvas, and mouseInBounds will be set to false.
	 * @property mouseY
	 * @type Number
	 * @readonly
	 **/
	p.mouseY = 0;

	// TODO: deprecated.
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the "{{#crossLink "Stage/stagemousemove:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseMove
	 * @type Function
	 * @deprecated Use addEventListener and the "stagemousemove" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "Stage/stagemouseup:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseUp
	 * @type Function
	 * @deprecated Use addEventListener and the "stagemouseup" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "Stage/stagemousedown:event"}}{{/crossLink}}
	 * event.
	 * @property onMouseDown
	 * @type Function
	 * @deprecated Use addEventListener and the "stagemousedown" event.
	 */

	// TODO: deprecated.
	/**
	 * Indicates whether this stage should use the {{#crossLink "DisplayObject/snapToPixel"}}{{/crossLink}} property of
	 * display objects when rendering them.
	 * @property snapToPixelEnabled
	 * @type Boolean
	 * @default false
	 * @deprecated Hardware acceleration makes this not beneficial
	 **/
	p.snapToPixelEnabled = false;

	/**
	 * Indicates whether the mouse is currently within the bounds of the canvas.
	 * @property mouseInBounds
	 * @type Boolean
	 * @default false
	 **/
	p.mouseInBounds = false;

	/**
	 * If true, tick callbacks will be called on all display objects on the stage prior to rendering to the canvas.
	 * @property tickOnUpdate
	 * @type Boolean
	 * @default true
	 **/
	p.tickOnUpdate = true;

	/**
	 * If true, mouse move events will continue to be called when the mouse leaves the target canvas. See
	 * {{#crossLink "Stage/mouseInBounds:property"}}{{/crossLink}}, and {{#crossLink "MouseEvent"}}{{/crossLink}}
	 * x/y/rawX/rawY.
	 * @property mouseMoveOutside
	 * @type Boolean
	 * @default false
	 **/
	p.mouseMoveOutside = false;
	
	// TODO: confirm naming and inclusion.
	/**
	 * NOTE: this name is not final. Feedback is appreciated.
	 *
	 * The stage assigned to this property will have mouse interactions relayed to it after this stage handles them.
	 * This can be useful in cases where you have multiple canvases layered on top of one another and want your mouse
	 * events to pass through. For example, this would relay mouse events from topStage to bottomStage:
	 *
	 *      topStage.nextStage = bottomStage;
	 *
	 * Note that each stage handles the interactions independently. As such, you could have a click register on an
	 * object in the top stage, and another click register in the bottom stage. Consider using a single canvas with
	 * cached {{#crossLink "Container"}}{{/crossLink}} instances instead of multiple canvases.
	 *
	 * MouseOver, MouseOut, RollOver, and RollOut interactions will not be passed through. They must be enabled using
	 * {{#crossLink "Stage/enableMouseOver"}}{{/crossLink}} for each stage individually.
	 * 
	 * In most instances, you will also want to disable DOM events for the next stage to avoid duplicate interactions.
	 * myNextStage.enableDOMEvents(false);
	 * 
	 * @property nextStage
	 * @type Stage
	 **/
	p.nextStage = null;

	/**
	 * The hitArea property is not supported for Stage.
	 * @property hitArea
	 * @type {DisplayObject}
	 * @default null
	 */

// private properties:

	/**
	 * Holds objects with data for each active pointer id. Each object has the following properties:
	 * x, y, event, target, overTarget, overX, overY, inBounds, posEvtObj (native event that last updated position)
	 * @property _pointerData
	 * @type {Object}
	 * @private
	 */
	p._pointerData = null;

	/**
	 * Number of active pointers.
	 * @property _pointerCount
	 * @type {Object}
	 * @private
	 */
	p._pointerCount = 0;

	/**
	 * The ID of the primary pointer.
	 * @property _primaryPointerID
	 * @type {Object}
	 * @private
	 */
	p._primaryPointerID = null;

	/**
	 * @property _mouseOverIntervalID
	 * @protected
	 * @type Number
	 **/
	p._mouseOverIntervalID = null;

// constructor:
	/**
	 * @property DisplayObject_initialize
	 * @type Function
	 * @private
	 **/
	p.Container_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @param {HTMLCanvasElement | String | Object} canvas A canvas object, or the string id of a canvas object in the current document.
	 * @protected
	 **/
	p.initialize = function(canvas) {
		this.Container_initialize();
		this.canvas = (typeof canvas == "string") ? document.getElementById(canvas) : canvas;
		this._pointerData = {};
		this.enableDOMEvents(true);
	};

// public methods:

	/**
	 * Each time the update method is called, the stage will tick all descendants (see: {{#crossLink "DisplayObject/tick"}}{{/crossLink}})
	 * and then render the display list to the canvas. Any parameters passed to `update()` will be passed on to any
	 * {{#crossLink "DisplayObject/tick:event"}}{{/crossLink}} event handlers.
	 *
	 * Some time-based features in EaselJS (for example {{#crossLink "Sprite/framerate"}}{{/crossLink}} require that
	 * a tick event object (or equivalent) be passed as the first parameter to update(). For example:
	 *
	 *      Ticker.addEventListener("tick", handleTick);
	 * 	    function handleTick(evtObj) {
	 * 	     	// do some work here, then update the stage, passing through the event object:
	 * 	    	myStage.update(evtObj);
	 * 	    }
	 *
	 * @method update
	 * @param {*} [params]* Params to include when ticking descendants. The first param should usually be a tick event.
	 **/
	p.update = function(params) {
		if (!this.canvas) { return; }
		if (this.tickOnUpdate) {
			this.dispatchEvent("tickstart");  // TODO: make cancellable?
			this.tickEnabled&&this._tick((arguments.length ? arguments : null));
			this.dispatchEvent("tickend");
		}
		this.dispatchEvent("drawstart"); // TODO: make cancellable?
		Stage._snapToPixelEnabled = this.snapToPixelEnabled;
		if (this.autoClear) { this.clear(); }
		var ctx = this.canvas.getContext("2d");
		ctx.save();
		this.updateContext(ctx);
		this.draw(ctx, false);
		ctx.restore();
		this.dispatchEvent("drawend");
	};

	/**
	 * Default event handler that calls the Stage {{#crossLink "Stage/update"}}{{/crossLink}} method when a {{#crossLink "DisplayObject/tick:event"}}{{/crossLink}}
	 * event is received. This allows you to register a Stage instance as a event listener on {{#crossLink "Ticker"}}{{/crossLink}}
	 * directly, using:
	 *
	 *      Ticker.addEventListener("tick", myStage");
	 *
	 * Note that if you subscribe to ticks using this pattern, then the tick event object will be passed through to
	 * display object tick handlers, instead of <code>delta</code> and <code>paused</code> parameters.
	 * @property handleEvent
	 * @type Function
	 **/
	p.handleEvent = function(evt) {
		if (evt.type == "tick") { this.update(evt); }
	};

	/**
	 * Clears the target canvas. Useful if {{#crossLink "Stage/autoClear:property"}}{{/crossLink}} is set to `false`.
	 * @method clear
	 **/
	p.clear = function() {
		if (!this.canvas) { return; }
		var ctx = this.canvas.getContext("2d");
		ctx.setTransform(1, 0, 0, 1, 0, 0);
		ctx.clearRect(0, 0, this.canvas.width+1, this.canvas.height+1);
	};

	/**
	 * Returns a data url that contains a Base64-encoded image of the contents of the stage. The returned data url can
	 * be specified as the src value of an image element.
	 * @method toDataURL
	 * @param {String} backgroundColor The background color to be used for the generated image. The value can be any value HTML color
	 * value, including HEX colors, rgb and rgba. The default value is a transparent background.
	 * @param {String} mimeType The MIME type of the image format to be create. The default is "image/png". If an unknown MIME type
	 * is passed in, or if the browser does not support the specified MIME type, the default value will be used.
	 * @return {String} a Base64 encoded image.
	 **/
	p.toDataURL = function(backgroundColor, mimeType) {
		if(!mimeType) {
			mimeType = "image/png";
		}

		var ctx = this.canvas.getContext('2d');
		var w = this.canvas.width;
		var h = this.canvas.height;

		var data;

		if(backgroundColor) {

			//get the current ImageData for the canvas.
			data = ctx.getImageData(0, 0, w, h);

			//store the current globalCompositeOperation
			var compositeOperation = ctx.globalCompositeOperation;

			//set to draw behind current content
			ctx.globalCompositeOperation = "destination-over";

			//set background color
			ctx.fillStyle = backgroundColor;

			//draw background on entire canvas
			ctx.fillRect(0, 0, w, h);
		}

		//get the image data from the canvas
		var dataURL = this.canvas.toDataURL(mimeType);

		if(backgroundColor) {
			//clear the canvas
			ctx.clearRect (0, 0, w+1, h+1);

			//restore it with original settings
			ctx.putImageData(data, 0, 0);

			//reset the globalCompositeOperation to what it was
			ctx.globalCompositeOperation = compositeOperation;
		}

		return dataURL;
	};

	/**
	 * Enables or disables (by passing a frequency of 0) mouse over ({{#crossLink "DisplayObject/mouseover:event"}}{{/crossLink}}
	 * and {{#crossLink "DisplayObject/mouseout:event"}}{{/crossLink}}) and roll over events ({{#crossLink "DisplayObject/rollover:event"}}{{/crossLink}}
	 * and {{#crossLink "DisplayObject/rollout:event"}}{{/crossLink}}) for this stage's display list. These events can
	 * be expensive to generate, so they are disabled by default. The frequency of the events can be controlled
	 * independently of mouse move events via the optional `frequency` parameter.
	 *
	 * <h4>Example</h4>
	 *      var stage = new createjs.Stage("canvasId");
	 *      stage.enableMouseOver(10); // 10 updates per second
	 *
	 * @method enableMouseOver
	 * @param {Number} [frequency=20] Optional param specifying the maximum number of times per second to broadcast
	 * mouse over/out events. Set to 0 to disable mouse over events completely. Maximum is 50. A lower frequency is less
	 * responsive, but uses less CPU.
	 **/
	p.enableMouseOver = function(frequency) {
		if (this._mouseOverIntervalID) {
			clearInterval(this._mouseOverIntervalID);
			this._mouseOverIntervalID = null;
			if (frequency == 0) {
				this._testMouseOver(true);
			}
		}
		if (frequency == null) { frequency = 20; }
		else if (frequency <= 0) { return; }
		var o = this;
		this._mouseOverIntervalID = setInterval(function(){ o._testMouseOver(); }, 1000/Math.min(50,frequency));
	};

	/**
	 * Enables or disables the event listeners that stage adds to DOM elements (window, document and canvas). It is good
	 * practice to disable events when disposing of a Stage instance, otherwise the stage will continue to receive
	 * events from the page.
	 *
	 * When changing the canvas property you must disable the events on the old canvas, and enable events on the
	 * new canvas or mouse events will not work as expected. For example:
	 *
	 *      myStage.enableDOMEvents(false);
	 *      myStage.canvas = anotherCanvas;
	 *      myStage.enableDOMEvents(true);
	 *
	 * @method enableDOMEvents
	 * @param {Boolean} [enable=true] Indicates whether to enable or disable the events. Default is true.
	 **/
	p.enableDOMEvents = function(enable) {
		if (enable == null) { enable = true; }
		var n, o, ls = this._eventListeners;
		if (!enable && ls) {
			for (n in ls) {
				o = ls[n];
				o.t.removeEventListener(n, o.f, false);
			}
			this._eventListeners = null;
		} else if (enable && !ls && this.canvas) {
			var t = window.addEventListener ? window : document;
			var _this = this;
			ls = this._eventListeners = {};
			ls["mouseup"] = {t:t, f:function(e) { _this._handleMouseUp(e)} };
			ls["mousemove"] = {t:t, f:function(e) { _this._handleMouseMove(e)} };
			ls["dblclick"] = {t:this.canvas, f:function(e) { _this._handleDoubleClick(e)} };
			ls["mousedown"] = {t:this.canvas, f:function(e) { _this._handleMouseDown(e)} };

			for (n in ls) {
				o = ls[n];
				o.t.addEventListener(n, o.f, false);
			}
		}
	};

	/**
	 * Returns a clone of this Stage.
	 * @method clone
	 * @return {Stage} A clone of the current Container instance.
	 **/
	p.clone = function() {
		var o = new Stage(null);
		this.cloneProps(o);
		return o;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Stage (name="+  this.name +")]";
	};

	// private methods:

	/**
	 * @method _getElementRect
	 * @protected
	 * @param {HTMLElement} e
	 **/
	p._getElementRect = function(e) {
		var bounds;
		try { bounds = e.getBoundingClientRect(); } // this can fail on disconnected DOM elements in IE9
		catch (err) { bounds = {top: e.offsetTop, left: e.offsetLeft, width:e.offsetWidth, height:e.offsetHeight}; }

		var offX = (window.pageXOffset || document.scrollLeft || 0) - (document.clientLeft || document.body.clientLeft || 0);
		var offY = (window.pageYOffset || document.scrollTop || 0) - (document.clientTop  || document.body.clientTop  || 0);

		var styles = window.getComputedStyle ? getComputedStyle(e) : e.currentStyle; // IE <9 compatibility.
		var padL = parseInt(styles.paddingLeft)+parseInt(styles.borderLeftWidth);
		var padT = parseInt(styles.paddingTop)+parseInt(styles.borderTopWidth);
		var padR = parseInt(styles.paddingRight)+parseInt(styles.borderRightWidth);
		var padB = parseInt(styles.paddingBottom)+parseInt(styles.borderBottomWidth);

		// note: in some browsers bounds properties are read only.
		return {
			left: bounds.left+offX+padL,
			right: bounds.right+offX-padR,
			top: bounds.top+offY+padT,
			bottom: bounds.bottom+offY-padB
		}
	};

	/**
	 * @method _getPointerData
	 * @protected
	 * @param {Number} id
	 **/
	p._getPointerData = function(id) {
		var data = this._pointerData[id];
		if (!data) {
			data = this._pointerData[id] = {x:0,y:0};
			// if it's the first new touch, then make it the primary pointer id:
			if (this._primaryPointerID == null) { this._primaryPointerID = id; }
			// if it's the mouse (id == -1) or the first new touch, then make it the primary pointer id:
		    if (this._primaryPointerID == null || this._primaryPointerID == -1) { this._primaryPointerID = id; }
		}
		return data;
	};

	/**
	 * @method _handleMouseMove
	 * @protected
	 * @param {MouseEvent} e
	 **/
	p._handleMouseMove = function(e) {
		if(!e){ e = window.event; }
		this._handlePointerMove(-1, e, e.pageX, e.pageY);
	};

	/**
	 * @method _handlePointerMove
	 * @protected
	 * @param {Number} id
	 * @param {Event} e
	 * @param {Number} pageX
	 * @param {Number} pageY
	 **/
	p._handlePointerMove = function(id, e, pageX, pageY) {
		if (!this.canvas) { return; }
		var o = this._getPointerData(id);

		var inBounds = o.inBounds;
		this._updatePointerPosition(id, e, pageX, pageY);
		if (!inBounds && !o.inBounds && !this.mouseMoveOutside) { return; }
		if (id == -1 && o.inBounds == !inBounds) {
			this._dispatchMouseEvent(this, (inBounds ? "mouseleave" : "mouseenter"), false, id, o, e);
		}
		
		this._dispatchMouseEvent(this, "stagemousemove", false, id, o, e);
		this._dispatchMouseEvent(o.target, "pressmove", true, id, o, e);

		// TODO: deprecated:
		var oEvent = o.event;
		if (oEvent && oEvent.hasEventListener("mousemove")) {
			// this doesn't use _dispatchMouseEvent because it requires re-targeting.
			oEvent.dispatchEvent(new createjs.MouseEvent("mousemove", false, false, o.x, o.y, e, id, (id == this._primaryPointerID), o.rawX, o.rawY), o.target);
		}

		this.nextStage&&this.nextStage._handlePointerMove(id, e, pageX, pageY);
	};

	/**
	 * @method _updatePointerPosition
	 * @protected
	 * @param {Number} id
	 * @param {Event} e
	 * @param {Number} pageX
	 * @param {Number} pageY
	 **/
	p._updatePointerPosition = function(id, e, pageX, pageY) {
		var rect = this._getElementRect(this.canvas);
		pageX -= rect.left;
		pageY -= rect.top;

		var w = this.canvas.width;
		var h = this.canvas.height;
		pageX /= (rect.right-rect.left)/w;
		pageY /= (rect.bottom-rect.top)/h;
		var o = this._getPointerData(id);
		if (o.inBounds = (pageX >= 0 && pageY >= 0 && pageX <= w-1 && pageY <= h-1)) {
			o.x = pageX;
			o.y = pageY;
		} else if (this.mouseMoveOutside) {
			o.x = pageX < 0 ? 0 : (pageX > w-1 ? w-1 : pageX);
			o.y = pageY < 0 ? 0 : (pageY > h-1 ? h-1 : pageY);
		}

		o.posEvtObj = e;
		o.rawX = pageX;
		o.rawY = pageY;

		if (id == this._primaryPointerID) {
			this.mouseX = o.x;
			this.mouseY = o.y;
			this.mouseInBounds = o.inBounds;
		}
	};

	/**
	 * @method _handleMouseUp
	 * @protected
	 * @param {MouseEvent} e
	 **/
	p._handleMouseUp = function(e) {
		this._handlePointerUp(-1, e, false);
	};

	/**
	 * @method _handlePointerUp
	 * @protected
	 * @param {Number} id
	 * @param {Event} e
	 * @param {Boolean} clear
	 **/
	p._handlePointerUp = function(id, e, clear) {
		var o = this._getPointerData(id);

		this._dispatchMouseEvent(this, "stagemouseup", false, id, o, e);

		var oTarget = o.target;
		if (oTarget) {
			if (this._getObjectsUnderPoint(o.x, o.y, null, true) == oTarget) {
				this._dispatchMouseEvent(oTarget, "click", true, id, o, e);
			}
			this._dispatchMouseEvent(oTarget, "pressup", true, id, o, e);
		}

		// TODO: deprecated:
		var oEvent = o.event;
		if (oEvent && oEvent.hasEventListener("mouseup")) {
			// this doesn't use _dispatchMouseEvent because it requires re-targeting.
			oEvent.dispatchEvent(new createjs.MouseEvent("mouseup", false, false, o.x, o.y, e, id, (id==this._primaryPointerID), o.rawX, o.rawY), oTarget);
		}

		if (clear) {
			if (id==this._primaryPointerID) { this._primaryPointerID = null; }
			delete(this._pointerData[id]);
		} else { o.event = o.target = null; }

		this.nextStage&&this.nextStage._handlePointerUp(id, e, clear);
	};

	/**
	 * @method _handleMouseDown
	 * @protected
	 * @param {MouseEvent} e
	 **/
	p._handleMouseDown = function(e) {
		this._handlePointerDown(-1, e, e.pageX, e.pageY);
	};

	/**
	 * @method _handlePointerDown
	 * @protected
	 * @param {Number} id
	 * @param {Event} e
	 * @param {Number} pageX
	 * @param {Number} pageY
	 **/
	p._handlePointerDown = function(id, e, pageX, pageY) {
		if (pageY != null) { this._updatePointerPosition(id, e, pageX, pageY); }
		var o = this._getPointerData(id);

		this._dispatchMouseEvent(this, "stagemousedown", false, id, o, e);

		o.target = this._getObjectsUnderPoint(o.x, o.y, null, true);
		// TODO: holding onto the event is deprecated:
		o.event =  this._dispatchMouseEvent(o.target, "mousedown", true, id, o, e);

		this.nextStage&&this.nextStage._handlePointerDown(id, e, pageX, pageY);
	};

	/**
	 * @method _testMouseOver
	 * @param {Boolean} clear If true, clears the mouseover / rollover (ie. no target)
	 * @protected
	 **/
	p._testMouseOver = function(clear) {
		// only update if the mouse position has changed. This provides a lot of optimization, but has some trade-offs.
		if (this._primaryPointerID != -1 || (!clear && this.mouseX == this._mouseOverX && this.mouseY == this._mouseOverY && this.mouseInBounds)) { return; }
		var o = this._getPointerData(-1);
		var e = o.posEvtObj;
		var target, common = -1, cursor="", t, i, l;
		
		if (clear || this.mouseInBounds && e && e.target == this.canvas) {
			target = this._getObjectsUnderPoint(this.mouseX, this.mouseY, null, true);
			this._mouseOverX = this.mouseX;
			this._mouseOverY = this.mouseY;
		}

		var oldList = this._mouseOverTarget||[];
		var oldTarget = oldList[oldList.length-1];
		var list = this._mouseOverTarget = [];

		// generate ancestor list and check for cursor:
		t = target;
		while (t) {
			list.unshift(t);
			if (t.cursor != null) { cursor = t.cursor; }
			t = t.parent;
		}
		this.canvas.style.cursor = cursor;

		// find common ancestor:
		for (i=0,l=list.length; i<l; i++) {
			if (list[i] != oldList[i]) { break; }
			common = i;
		}

		if (oldTarget != target) {
			this._dispatchMouseEvent(oldTarget, "mouseout", true, -1, o, e);
		}

		for (i=oldList.length-1; i>common; i--) {
			this._dispatchMouseEvent(oldList[i], "rollout", false, -1, o, e);
		}

		for (i=list.length-1; i>common; i--) {
			this._dispatchMouseEvent(list[i], "rollover", false, -1, o, e);
		}

		if (oldTarget != target) {
			this._dispatchMouseEvent(target, "mouseover", true, -1, o, e);
		}

	};

	/**
	 * @method _handleDoubleClick
	 * @protected
	 * @param {MouseEvent} e
	 **/
	p._handleDoubleClick = function(e) {
		var o = this._getPointerData(-1);
		var target = this._getObjectsUnderPoint(o.x, o.y, null, true);
		this._dispatchMouseEvent(target, "dblclick", true, -1, o, e);

		this.nextStage&&this.nextStage._handleDoubleClick(e);
	};

	/**
	 * @method _dispatchMouseEvent
	 * @protected
	 * @param {DisplayObject} target
	 * @param {String} type
	 * @param {Boolean} bubbles
	 * @param {Number} pointerId
	 * @param {Object} o
	 * @param {MouseEvent} [nativeEvent]
	 **/
	p._dispatchMouseEvent = function(target, type, bubbles, pointerId, o, nativeEvent) {
		// TODO: might be worth either reusing MouseEvent instances, or adding a willTrigger method to avoid GC.
		if (!target || (!bubbles && !target.hasEventListener(type))) { return; }
		/*
		// TODO: account for stage transformations:
		this._mtx = this.getConcatenatedMatrix(this._mtx).invert();
		var pt = this._mtx.transformPoint(o.x, o.y);
		var evt = new createjs.MouseEvent(type, bubbles, false, pt.x, pt.y, nativeEvent, pointerId, pointerId==this._primaryPointerID, o.rawX, o.rawY);
		*/
		var evt = new createjs.MouseEvent(type, bubbles, false, o.x, o.y, nativeEvent, pointerId, pointerId==this._primaryPointerID, o.rawX, o.rawY);
		target.dispatchEvent(evt);
		// TODO: returning evt is deprecated:
		return evt;
	};

createjs.Stage = Stage;
}());
/*
* Bitmap
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
* 
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
* 
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
* @module EaselJS
*/

// namespace:
this.createjs = this.createjs||{};

(function() {

/**
 * A Bitmap represents an Image, Canvas, or Video in the display list. A Bitmap can be instantiated using an existing
 * HTML element, or a string.
 *
 * <h4>Example</h4>
 *      var bitmap = new createjs.Bitmap("imagePath.jpg");
 *
 * <strong>Notes:</strong>
 * <ol>
 *     <li>When a string path or image tag that is not yet loaded is used, the stage may need to be redrawn before it
 *      will be displayed.</li>
 *     <li>Bitmaps with an SVG source currently will not respect an alpha value other than 0 or 1. To get around this,
 *     the Bitmap can be cached.</li>
 *     <li>Bitmaps with an SVG source will taint the canvas with cross-origin data, which prevents interactivity. This
 *     happens in all browsers except recent Firefox builds.</li>
 *     <li>Images loaded cross-origin will throw cross-origin security errors when interacted with using a mouse, using
 *     methods such as `getObjectUnderPoint`, or using filters, or caching. You can get around this by setting
 *     `crossOrigin` flags on your images before passing them to EaselJS, eg: `img.crossOrigin="Anonymous";`</li>
 * </ol>
 *
 * @class Bitmap
 * @extends DisplayObject
 * @constructor
 * @param {Image | HTMLCanvasElement | HTMLVideoElement | String} imageOrUri The source object or URI to an image to
 * display. This can be either an Image, Canvas, or Video object, or a string URI to an image file to load and use.
 * If it is a URI, a new Image object will be constructed and assigned to the .image property.
 **/
var Bitmap = function(imageOrUri) {
  this.initialize(imageOrUri);
};
var p = Bitmap.prototype = new createjs.DisplayObject();

// public properties:
	/**
	 * The image to render. This can be an Image, a Canvas, or a Video.
	 * @property image
	 * @type Image | HTMLCanvasElement | HTMLVideoElement
	 **/
	p.image = null;
	
	/**
	 * Whether or not the Bitmap should be draw to the canvas at whole pixel coordinates.
	 * @property snapToPixel
	 * @type Boolean
	 * @default true
	 **/
	p.snapToPixel = true;

	/**
	 * Specifies an area of the source image to draw. If omitted, the whole image will be drawn.
	 * @property sourceRect
	 * @type Rectangle
	 * @default null
	 */
	p.sourceRect = null;
	
	// constructor:

	/**
	 * @property DisplayObject_initialize
	 * @type Function
	 * @private
	 **/
	p.DisplayObject_initialize = p.initialize;

	/** 
	 * Initialization method.
	 * @method initialize
	 * @param {Image | HTMLCanvasElement | HTMLVideoElement | String} imageOrUri The source object or URI to an image to
	 * display. This can be either an Image, Canvas, or Video object, or a string URI to an image file to load and use.
	 * If it is a URI, a new Image object will be constructed and assigned to the `.image` property.
	 * @protected
	 **/
	p.initialize = function(imageOrUri) {
		this.DisplayObject_initialize();
		if (typeof imageOrUri == "string") {
			this.image = document.createElement("img");
			this.image.src = imageOrUri;
		} else {
			this.image = imageOrUri;
		}
	};
	
// public methods:

	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the display object would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		var hasContent = this.cacheCanvas || (this.image && (this.image.complete || this.image.getContext || this.image.readyState >= 2));
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0 && hasContent);
	};

	/**
	 * @property DisplayObject_draw
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_draw = p.draw;

	/**
	 * Draws the display object into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 *
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} [ignoreCache=false] Indicates whether the draw operation should ignore any current cache.
	 * For example, used for drawing the cache (to prevent it from simply drawing an existing cache back
	 * into itself).
	 * @return {Boolean}
	 **/
	p.draw = function(ctx, ignoreCache) {
		if (this.DisplayObject_draw(ctx, ignoreCache)) { return true; }
		var rect = this.sourceRect;
		if (rect) {
			ctx.drawImage(this.image, rect.x, rect.y, rect.width, rect.height, 0, 0, rect.width, rect.height);
		} else {
			ctx.drawImage(this.image, 0, 0);
		}
		return true;
	};
	
	//Note, the doc sections below document using the specified APIs (from DisplayObject)  from
	//Bitmap. This is why they have no method implementations.
	
	/**
	 * Because the content of a Bitmap is already in a simple format, cache is unnecessary for Bitmap instances.
	 * You should <b>not</b> cache Bitmap instances as it can degrade performance.
	 *
	 * <strong>However: If you want to use a filter on a Bitmap, you <em>MUST</em> cache it, or it will not work.</strong>
	 * To see the API for caching, please visit the DisplayObject {{#crossLink "DisplayObject/cache"}}{{/crossLink}}
	 * method.
	 * @method cache
	 **/
	
	/**
	 * Because the content of a Bitmap is already in a simple format, cache is unnecessary for Bitmap instances.
	 * You should <b>not</b> cache Bitmap instances as it can degrade performance.
	 *
	 * <strong>However: If you want to use a filter on a Bitmap, you <em>MUST</em> cache it, or it will not work.</strong>
	 * To see the API for caching, please visit the DisplayObject {{#crossLink "DisplayObject/cache"}}{{/crossLink}}
	 * method.
	 * @method updateCache
	 **/
	
	/**
	 * Because the content of a Bitmap is already in a simple format, cache is unnecessary for Bitmap instances.
	 * You should <b>not</b> cache Bitmap instances as it can degrade performance.
	 *
	 * <strong>However: If you want to use a filter on a Bitmap, you <em>MUST</em> cache it, or it will not work.</strong>
	 * To see the API for caching, please visit the DisplayObject {{#crossLink "DisplayObject/cache"}}{{/crossLink}}
	 * method.
	 * @method uncache
	 **/
	 
	/**
	 * @property DisplayObject_getBounds
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_getBounds = p.getBounds;

	/**
	 * Docced in superclass.
	 */
	p.getBounds = function() {
		var rect = this.DisplayObject_getBounds();
		if (rect) { return rect; }
		var o = this.sourceRect || this.image;
		var hasContent = (this.image && (this.image.complete || this.image.getContext || this.image.readyState >= 2));
		return hasContent ? this._rectangle.initialize(0, 0, o.width, o.height) : null;
	};
	
	/**
	 * Returns a clone of the Bitmap instance.
	 * @method clone
	 * @return {Bitmap} a clone of the Bitmap instance.
	 **/
	p.clone = function() {
		var o = new Bitmap(this.image);
		if (this.sourceRect) { o.sourceRect = this.sourceRect.clone(); }
		this.cloneProps(o);
		return o;
	};
	
	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Bitmap (name="+  this.name +")]";
	};

// private methods:

createjs.Bitmap = Bitmap;
}());/*
* Sprite
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Displays a frame or sequence of frames (ie. an animation) from a SpriteSheet instance. A sprite sheet is a series of
 * images (usually animation frames) combined into a single image. For example, an animation consisting of 8 100x100
 * images could be combined into a 400x200 sprite sheet (4 frames across by 2 high). You can display individual frames,
 * play frames as an animation, and even sequence animations together.
 *
 * See the {{#crossLink "SpriteSheet"}}{{/crossLink}} class for more information on setting up frames and animations.
 *
 * <h4>Example</h4>
 *      var instance = new createjs.Sprite(spriteSheet);
 *      instance.gotoAndStop("frameName");
 *
 * Until {{#crossLink "Sprite/gotoAndStop"}}{{/crossLink}} or {{#crossLink "Sprite/gotoAndPlay"}}{{/crossLink}} is called,
 * only the first defined frame defined in the sprite sheet will be displayed.
 *
 * @class Sprite
 * @extends DisplayObject
 * @constructor
 * @param {SpriteSheet} spriteSheet The SpriteSheet instance to play back. This includes the source image(s), frame
 * dimensions, and frame data. See {{#crossLink "SpriteSheet"}}{{/crossLink}} for more information.
 * @param {String|Number} frameOrAnimation The frame number or animation to play initially.
 **/
var Sprite = function(spriteSheet, frameOrAnimation) {
  this.initialize(spriteSheet, frameOrAnimation);
};
var p = Sprite.prototype = new createjs.DisplayObject();

// events:

	/**
	 * Dispatched when an animation reaches its ends.
	 * @event animationend
	 * @param {Object} target The object that dispatched the event.
	 * @param {String} type The event type.
	 * @param {String} name The name of the animation that just ended.
	 * @param {String} next The name of the next animation that will be played, or null. This will be the same as name if the animation is looping.
	 * @since 0.6.0
	 */

// public properties:
	// TODO: deprecated.
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "Sprite/animationend:event"}}{{/crossLink}}
	 * event.
	 * @property onAnimationEnd
	 * @type {Function}
	 * @deprecated Use addEventListener and the "animationend" event.
	 */

	/**
	 * The frame index that will be drawn when draw is called. Note that with some {{#crossLink "SpriteSheet"}}{{/crossLink}}
	 * definitions, this will advance non-sequentially. This will always be an integer value.
	 * @property currentFrame
	 * @type {Number}
	 * @default 0
	 * @readonly
	 **/
	p.currentFrame = 0;

	/**
	 * Returns the name of the currently playing animation.
	 * @property currentAnimation
	 * @type {String}
	 * @final
	 * @readonly
	 **/
	p.currentAnimation = null;

	/**
	 * Prevents the animation from advancing each tick automatically. For example, you could create a sprite
	 * sheet of icons, set paused to true, and display the appropriate icon by setting <code>currentFrame</code>.
	 * @property paused
	 * @type {Boolean}
	 * @default false
	 **/
	p.paused = true;

	/**
	 * The SpriteSheet instance to play back. This includes the source image, frame dimensions, and frame
	 * data. See {{#crossLink "SpriteSheet"}}{{/crossLink}} for more information.
	 * @property spriteSheet
	 * @type {SpriteSheet}
	 * @readonly
	 **/
	p.spriteSheet = null;

	/**
	 * Whether or not the image should be draw to the canvas at whole pixel coordinates.
	 * @property snapToPixel
	 * @type {Boolean}
	 * @default true
	 **/
	p.snapToPixel = true;

	/**
	 * @property offset
	 * @type {Number}
	 * @default 0
	 * @deprecated Not applicable to the new timing model in v0.7.0 and higher.
	 */
	p.offset = 0;

	/**
	 * Specifies the current frame index within the currently playing animation. When playing normally, this will increase
	 * from 0 to n-1, where n is the number of frames in the current animation.
	 *
	 * This could be a non-integer value if
	 * using time-based playback (see {{#crossLink "Sprite/framerate"}}{{/crossLink}}, or if the animation's speed is
	 * not an integer.
	 * @property currentAnimationFrame
	 * @type {Number}
	 * @default 0
	 **/
	p.currentAnimationFrame = 0;

	/**
	 * By default Sprite instances advance one frame per tick. Specifying a framerate for the Sprite (or its related
	 * SpriteSheet) will cause it to advance based on elapsed time between ticks as appropriate to maintain the target
	 * framerate.
	 *
	 * For example, if a Sprite with a framerate of 10 is placed on a Stage being updated at 40fps, then the Sprite will
	 * advance roughly one frame every 4 ticks. This will not be exact, because the time between each tick will
	 * vary slightly between frames.
	 *
	 * This feature is dependent on the tick event object (or an object with an appropriate "delta" property) being
	 * passed into {{#crossLink "Stage/update"}}{{/crossLink}}.
	 * @property framerate
	 * @type {Number}
	 * @default 0
	 **/
	p.framerate = 0;

// private properties:
	/**
	 * @property _advanceCount
	 * @protected
	 * @type {Number}
	 * @default 0
	 **/
	p._advanceCount = 0;

	/**
	 * @property _animation
	 * @protected
	 * @type {Object}
	 * @default null
	 **/
	p._animation = null;

	/**
	 * @property _animation
	 * @protected
	 * @type {Object}
	 * @default null
	 **/
	p._currentFrame = null;

// constructor:
	/**
	 * @property DisplayObject_initialize
	 * @type {Function}
	 * @private
	 **/
	p.DisplayObject_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	*/
	p.initialize = function(spriteSheet, frameOrAnimation) {
		this.DisplayObject_initialize();
		this.spriteSheet = spriteSheet;
		if (frameOrAnimation) { this.gotoAndPlay(frameOrAnimation); }
	};

	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the display object would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		var hasContent = this.cacheCanvas || this.spriteSheet.complete;
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0 && hasContent);
	};

	/**
	 * @property DisplayObject_draw
	 * @type {Function}
	 * @private
	 **/
	p.DisplayObject_draw = p.draw;

	/**
	 * Draws the display object into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} ignoreCache Indicates whether the draw operation should ignore any current cache.
	 * For example, used for drawing the cache (to prevent it from simply drawing an existing cache back
	 * into itself).
	 **/
	p.draw = function(ctx, ignoreCache) {
		if (this.DisplayObject_draw(ctx, ignoreCache)) { return true; }
		this._normalizeFrame();
		var o = this.spriteSheet.getFrame(this._currentFrame|0);
		if (!o) { return false; }
		var rect = o.rect;
		ctx.drawImage(o.image, rect.x, rect.y, rect.width, rect.height, -o.regX, -o.regY, rect.width, rect.height);
		return true;
	};

	//Note, the doc sections below document using the specified APIs (from DisplayObject)  from
	//Bitmap. This is why they have no method implementations.

	/**
	 * Because the content of a Bitmap is already in a simple format, cache is unnecessary for Bitmap instances.
	 * You should not cache Bitmap instances as it can degrade performance.
	 * @method cache
	 **/

	/**
	 * Because the content of a Bitmap is already in a simple format, cache is unnecessary for Bitmap instances.
	 * You should not cache Bitmap instances as it can degrade performance.
	 * @method updateCache
	 **/

	/**
	 * Because the content of a Bitmap is already in a simple format, cache is unnecessary for Bitmap instances.
	 * You should not cache Bitmap instances as it can degrade performance.
	 * @method uncache
	 **/

	/**
	 * Play (unpause) the current animation. The Sprite will be paused if either {{#crossLink "Sprite/stop"}}{{/crossLink}}
	 * or {{#crossLink "Sprite/gotoAndStop"}}{{/crossLink}} is called. Single frame animations will remain
	 * unchanged.
	 * @method play
	 **/
	p.play = function() {
		this.paused = false;
	};

	/**
	 * Stop playing a running animation. The Sprite will be playing if {{#crossLink "Sprite/gotoAndPlay"}}{{/crossLink}}
	 * is called. Note that calling {{#crossLink "Sprite/gotoAndPlay"}}{{/crossLink}} or {{#crossLink "Sprite/play"}}{{/crossLink}}
	 * will resume playback.
	 * @method stop
	 **/
	p.stop = function() {
		this.paused = true;
	};

	/**
	 * Sets paused to false and plays the specified animation name, named frame, or frame number.
	 * @method gotoAndPlay
	 * @param {String|Number} frameOrAnimation The frame number or animation name that the playhead should move to
	 * and begin playing.
	 **/
	p.gotoAndPlay = function(frameOrAnimation) {
		this.paused = false;
		this._goto(frameOrAnimation);
	};

	/**
	 * Sets paused to true and seeks to the specified animation name, named frame, or frame number.
	 * @method gotoAndStop
	 * @param {String|Number} frameOrAnimation The frame number or animation name that the playhead should move to
	 * and stop.
	 **/
	p.gotoAndStop = function(frameOrAnimation) {
		this.paused = true;
		this._goto(frameOrAnimation);
	};

	/**
	 * Advances the playhead. This occurs automatically each tick by default.
	 * @param [time] {Number} The amount of time in ms to advance by. Only applicable if framerate is set on the Sprite
	 * or its SpriteSheet.
	 * @method advance
	*/
	p.advance = function(time) {
		var speed = (this._animation&&this._animation.speed)||1;
		var fps = this.framerate || this.spriteSheet.framerate;
		var t = (fps && time != null) ? time/(1000/fps) : 1;
		if (this._animation) { this.currentAnimationFrame+=t*speed; }
		else { this._currentFrame+=t*speed; }
		this._normalizeFrame();
	};
	
	/**
	 * @property DisplayObject_getBounds
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_getBounds = p.getBounds; 
	
	/**
	 * Returns a {{#crossLink "Rectangle"}}{{/crossLink}} instance defining the bounds of the current frame relative to
	 * the origin. For example, a 90 x 70 frame with <code>regX=50</code> and <code>regY=40</code> would return a
	 * rectangle with [x=-50, y=-40, width=90, height=70]. This ignores transformations on the display object.
	 *
	 * Also see the SpriteSheet {{#crossLink "SpriteSheet/getFrameBounds"}}{{/crossLink}} method.
	 * @method getBounds
	 * @return {Rectangle} A Rectangle instance. Returns null if the frame does not exist, or the image is not fully
	 * loaded.
	 **/
	p.getBounds = function() {
		// TODO: should this normalizeFrame?
		return this.DisplayObject_getBounds() || this.spriteSheet.getFrameBounds(this.currentFrame, this._rectangle);
	};

	/**
	 * Returns a clone of the Sprite instance. Note that the same SpriteSheet is shared between cloned
	 * instances.
	 * @method clone
	 * @return {Sprite} a clone of the Sprite instance.
	 **/
	p.clone = function() {
		var o = new Sprite(this.spriteSheet);
		this.cloneProps(o);
		return o;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Sprite (name="+  this.name +")]";
	};

// private methods:
	/**
	 * @property DisplayObject__tick
	 * @type {Function}
	 * @private
	 **/
	p.DisplayObject__tick = p._tick;

	/**
	 * Advances the <code>currentFrame</code> if paused is not true. This is called automatically when the {{#crossLink "Stage"}}{{/crossLink}}
	 * ticks.
	 * @protected
	 * @method _tick
	 **/
	p._tick = function(params) {
		if (!this.paused) {
			this.advance(params&&params[0]&&params[0].delta);
		}
		this.DisplayObject__tick(params);
	};


	/**
	 * Normalizes the current frame, advancing animations and dispatching callbacks as appropriate.
	 * @protected
	 * @method _normalizeFrame
	 **/
	p._normalizeFrame = function() {
		var animation = this._animation;
		var paused = this.paused;
		var frame = this._currentFrame;
		var animFrame = this.currentAnimationFrame;
		var l;

		if (animation) {
			l = animation.frames.length;
			if ((animFrame|0) >= l) {
				var next = animation.next;
				if (this._dispatchAnimationEnd(animation, frame, paused, next, l-1)) {
					// something changed in the event stack.
				} else if (next) {
					// sequence. Automatically calls _normalizeFrame again.
					return this._goto(next, animFrame - l);
				} else {
					// end.
					this.paused = true;
					animFrame = this.currentAnimationFrame = animation.frames.length-1;
					this._currentFrame = animation.frames[animFrame];
				}
			} else {
				this._currentFrame = animation.frames[animFrame|0];
			}
		} else {
			l = this.spriteSheet.getNumFrames();
			if (frame >= l) {
				if (!this._dispatchAnimationEnd(animation, frame, paused, l-1)) {
					// looped.
					if ((this._currentFrame -= l) >= l) { return this._normalizeFrame(); }
				}
			}
		}
		this.currentFrame = this._currentFrame|0;
	};

	/**
	 * Dispatches the "animationend" event. Returns true if a handler changed the animation (ex. calling {{#crossLink "Sprite/stop"}}{{/crossLink}},
	 * {{#crossLink "Sprite/gotoAndPlay"}}{{/crossLink}}, etc.)
	 * @property _dispatchAnimationEnd
	 * @private
	 * @type {Function}
	 **/
	p._dispatchAnimationEnd = function(animation, frame, paused, next, end) {
		var name = animation ? animation.name : null;
		if (this.hasEventListener("animationend")) {
			var evt = new createjs.Event("animationend");
			evt.name = name;
			evt.next = next;
			this.dispatchEvent(evt);
		}
		// did the animation get changed in the event stack?:
		var changed = (this._animation != animation || this._currentFrame != frame);
		// if the animation hasn't changed, but the sprite was paused, then we want to stick to the last frame:
		if (!changed && !paused && this.paused) { this.currentAnimationFrame = end; changed = true; }
		return changed;
	};

	/**
	 * @property DisplayObject_cloneProps
	 * @private
	 * @type {Function}
	 **/
	p.DisplayObject_cloneProps = p.cloneProps;

	/**
	 * @method cloneProps
	 * @param {Text} o
	 * @protected
	 **/
	p.cloneProps = function(o) {
		this.DisplayObject_cloneProps(o);
		o.currentFrame = this.currentFrame;
		o._currentFrame = this._currentFrame;
		o.currentAnimation = this.currentAnimation;
		o.paused = this.paused;
		o._animation = this._animation;
		o.currentAnimationFrame = this.currentAnimationFrame;
		o.framerate = this.framerate;
	};

	/**
	 * Moves the playhead to the specified frame number or animation.
	 * @method _goto
	 * @param {String|Number} frameOrAnimation The frame number or animation that the playhead should move to.
	 * @param {Boolean} [frame] The frame of the animation to go to. Defaults to 0.
	 * @protected
	 **/
	p._goto = function(frameOrAnimation, frame) {
		if (isNaN(frameOrAnimation)) {
			var data = this.spriteSheet.getAnimation(frameOrAnimation);
			if (data) {
				this.currentAnimationFrame = frame||0;
				this._animation = data;
				this.currentAnimation = frameOrAnimation;
				this._normalizeFrame();
			}
		} else {
			this.currentAnimationFrame = 0;
			this.currentAnimation = this._animation = null;
			this._currentFrame = frameOrAnimation;
			this._normalizeFrame();
		}
	};

createjs.Sprite = Sprite;
}());
/*
* BitmapAnimation
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";
/**
 * Deprecated in favour of {{#crossLink "Sprite"}}{{/crossLink}}.
 *
 * @class BitmapAnimation
 * @extends DisplayObject
 * @constructor
 * @param {SpriteSheet} spriteSheet The SpriteSheet instance to play back. This includes the source image(s), frame
 * dimensions, and frame data. See {{#crossLink "SpriteSheet"}}{{/crossLink}} for more information.
 * @deprecated Renamed to Sprite. Will be removed in a future version.
 **/

var e = "BitmapAnimation is deprecated in favour of Sprite. See VERSIONS file for info on changes.";
if (!createjs.Sprite) { throw(e); }
(createjs.BitmapAnimation = function(spriteSheet) {
  console.log(e);
  this.initialize(spriteSheet);
}).prototype = new createjs.Sprite();
})();
/*
* Shape
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * A Shape allows you to display vector art in the display list. It composites a {{#crossLink "Graphics"}}{{/crossLink}}
 * instance which exposes all of the vector drawing methods. The Graphics instance can be shared between multiple Shape
 * instances to display the same vector graphics with different positions or transforms.
 *
 * If the vector art will not
 * change between draws, you may want to use the {{#crossLink "DisplayObject/cache"}}{{/crossLink}} method to reduce the
 * rendering cost.
 *
 * <h4>Example</h4>
 *      var graphics = new createjs.Graphics().beginFill("#ff0000").drawRect(0, 0, 100, 100);
 *      var shape = new createjs.Shape(graphics);
 *
 *      //Alternatively use can also use the graphics property of the Shape class to renderer the same as above.
 *      var shape = new createjs.Shape();
 *      shape.graphics.beginFill("#ff0000").drawRect(0, 0, 100, 100);
 *
 * @class Shape
 * @extends DisplayObject
 * @constructor
 * @param {Graphics} graphics Optional. The graphics instance to display. If null, a new Graphics instance will be created.
 **/
var Shape = function(graphics) {
  this.initialize(graphics);
}
var p = Shape.prototype = new createjs.DisplayObject();

// public properties:
	/**
	 * The graphics instance to display.
	 * @property graphics
	 * @type Graphics
	 **/
	p.graphics = null;

// constructor:
	/**
	 * @property DisplayObject_initialize
	 * @private
	 * @type Function
	 **/
	p.DisplayObject_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @param {Graphics} graphics
	 * @protected
	 **/
	p.initialize = function(graphics) {
		this.DisplayObject_initialize();
		this.graphics = graphics ? graphics : new createjs.Graphics();
	}

	/**
	 * Returns true or false indicating whether the Shape would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the Shape would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		var hasContent = this.cacheCanvas || (this.graphics && !this.graphics.isEmpty());
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0 && hasContent);
	};

	/**
	 * @property DisplayObject_draw
	 * @private
	 * @type Function
	 **/
	p.DisplayObject_draw = p.draw;

	/**
	 * Draws the Shape into the specified context ignoring its visible, alpha, shadow, and transform. Returns true if
	 * the draw was handled (useful for overriding functionality).
	 *
	 * <i>NOTE: This method is mainly for internal use, though it may be useful for advanced uses.</i>
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} [ignoreCache=false] Indicates whether the draw operation should ignore any current cache. For example,
	 * used for drawing the cache (to prevent it from simply drawing an existing cache back into itself).
	 * @return {Boolean}
	 **/
	p.draw = function(ctx, ignoreCache) {
		if (this.DisplayObject_draw(ctx, ignoreCache)) { return true; }
		this.graphics.draw(ctx);
		return true;
	}

	/**
	 * Returns a clone of this Shape. Some properties that are specific to this instance's current context are reverted to
	 * their defaults (for example .parent).
	 * @method clone
	 * @param {Boolean} recursive If true, this Shape's {{#crossLink "Graphics"}}{{/crossLink}} instance will also be
	 * cloned. If false, the Graphics instance will be shared with the new Shape.
	 **/
	p.clone = function(recursive) {
		var o = new Shape((recursive && this.graphics) ? this.graphics.clone() : this.graphics);
		this.cloneProps(o);
		return o;
	}

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Shape (name="+  this.name +")]";
	}

createjs.Shape = Shape;
}());
/*
* Text
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Display one or more lines of dynamic text (not user editable) in the display list. Line wrapping support (using the
 * lineWidth) is very basic, wrapping on spaces and tabs only. Note that as an alternative to Text, you can position HTML
 * text above or below the canvas relative to items in the display list using the {{#crossLink "DisplayObject/localToGlobal"}}{{/crossLink}}
 * method, or using {{#crossLink "DOMElement"}}{{/crossLink}}.
 *
 * <b>Please note that Text does not support HTML text, and can only display one font style at a time.</b> To use
 * multiple font styles, you will need to create multiple text instances, and position them manually.
 *
 * <h4>Example</h4>
 *      var text = new createjs.Text("Hello World", "20px Arial", "#ff7700");
 *      text.x = 100;
 *      text.textBaseline = "alphabetic";
 *
 * CreateJS Text supports web fonts (the same rules as Canvas). The font must be loaded and supported by the browser
 * before it can be displayed.
 *
 * <strong>Note:</strong> Text can be expensive to generate, so cache instances where possible. Be aware that not all
 * browsers will render Text exactly the same.
 * @class Text
 * @extends DisplayObject
 * @constructor
 * @param {String} [text] The text to display.
 * @param {String} [font] The font style to use. Any valid value for the CSS font attribute is acceptable (ex. "bold
 * 36px Arial").
 * @param {String} [color] The color to draw the text in. Any valid value for the CSS color attribute is acceptable (ex.
 * "#F00", "red", or "#FF0000").
 **/
var Text = function(text, font, color) {
  this.initialize(text, font, color);
};
var p = Text.prototype = new createjs.DisplayObject();

	/**
	 * @property _workingContext
	 * @type CanvasRenderingContext2D
	 * @private
	 **/
	var canvas = (createjs.createCanvas?createjs.createCanvas():document.createElement("canvas"));
	if (canvas.getContext) { Text._workingContext = canvas.getContext("2d"); canvas.width = canvas.height = 1; }
	
// static properties:
	/**
	 * Lookup table for the ratio to offset bounds x calculations based on the textAlign property.
	 * @property H_OFFSETS
	 * @type Object
	 * @protected
	 * @static
	 **/
	Text.H_OFFSETS = {start: 0, left: 0, center: -0.5, end: -1, right: -1};
	
	/**
	 * Lookup table for the ratio to offset bounds y calculations based on the textBaseline property.
	 * @property H_OFFSETS
	 * @type Object
	 * @protected
	 * @static
	 **/
	Text.V_OFFSETS = {top: 0, hanging: -0.01, middle: -0.4, alphabetic: -0.8, ideographic: -0.85, bottom: -1};

// public properties:
	/**
	 * The text to display.
	 * @property text
	 * @type String
	 **/
	p.text = "";

	/**
	 * The font style to use. Any valid value for the CSS font attribute is acceptable (ex. "bold 36px Arial").
	 * @property font
	 * @type String
	 **/
	p.font = null;

	/**
	 * The color to draw the text in. Any valid value for the CSS color attribute is acceptable (ex. "#F00"). Default is "#000".
	 * It will also accept valid canvas fillStyle values.
	 * @property color
	 * @type String
	 **/
	p.color = null;

	/**
	 * The horizontal text alignment. Any of "start", "end", "left", "right", and "center". For detailed
	 * information view the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#text-styles">
	 * whatwg spec</a>. Default is "left".
	 * @property textAlign
	 * @type String
	 **/
	p.textAlign = "left";

	/**
	 * The vertical alignment point on the font. Any of "top", "hanging", "middle", "alphabetic", "ideographic", or
	 * "bottom". For detailed information view the <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#text-styles">
	 * whatwg spec</a>. Default is "top".
	 * @property textBaseline
	 * @type String
	*/
	p.textBaseline = "top";

	/**
	 * The maximum width to draw the text. If maxWidth is specified (not null), the text will be condensed or
	 * shrunk to make it fit in this width. For detailed information view the
	 * <a href="http://www.whatwg.org/specs/web-apps/current-work/multipage/the-canvas-element.html#text-styles">
	 * whatwg spec</a>.
	 * @property maxWidth
	 * @type Number
	*/
	p.maxWidth = null;

	/**
	 * If greater than 0, the text will be drawn as a stroke (outline) of the specified width.
	 * @property outline
	 * @type Number
	 **/
	p.outline = 0;

	/**
	 * Indicates the line height (vertical distance between baselines) for multi-line text. If null or 0,
	 * the value of getMeasuredLineHeight is used.
	 * @property lineHeight
	 * @type Number
	 **/
	p.lineHeight = 0;

	/**
	 * Indicates the maximum width for a line of text before it is wrapped to multiple lines. If null,
	 * the text will not be wrapped.
	 * @property lineWidth
	 * @type Number
	 **/
	p.lineWidth = null;

// private properties:

// constructor:
	/**
	 * @property DisplayObject_initialize
	 * @private
	 * @type Function
	 **/
	p.DisplayObject_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @param {String} [text] The text to display.
	 * @param {String} [font] The font style to use. Any valid value for the CSS font attribute is acceptable (ex. "bold
	 * 36px Arial").
	 * @param {String} [color] The color to draw the text in. Any valid value for the CSS color attribute is acceptable (ex.
	 * "#F00", "red", or "#FF0000").
	 * @protected
	*/
	p.initialize = function(text, font, color) {
		this.DisplayObject_initialize();
		this.text = text;
		this.font = font;
		this.color = color;
	};

	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Whether the display object would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		var hasContent = this.cacheCanvas || (this.text != null && this.text !== "");
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0 && hasContent);
	};

	/**
	 * @property DisplayObject_draw
	 * @private
	 * @type Function
	 **/
	p.DisplayObject_draw = p.draw;

	/**
	 * Draws the Text into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} ignoreCache Indicates whether the draw operation should ignore any current cache.
	 * For example, used for drawing the cache (to prevent it from simply drawing an existing cache back
	 * into itself).
	 **/
	p.draw = function(ctx, ignoreCache) {
		if (this.DisplayObject_draw(ctx, ignoreCache)) { return true; }

		var col = this.color || "#000";
		if (this.outline) { ctx.strokeStyle = col; ctx.lineWidth = this.outline*1; }
		else { ctx.fillStyle = col; }
		
		this._drawText(this._prepContext(ctx));
		return true;
	};

	/**
	 * Returns the measured, untransformed width of the text without wrapping. Use getBounds for a more robust value.
	 * @method getMeasuredWidth
	 * @return {Number} The measured, untransformed width of the text.
	 **/
	p.getMeasuredWidth = function() {
		return this._prepContext(Text._workingContext).measureText(this.text).width;
	};

	/**
	 * Returns an approximate line height of the text, ignoring the lineHeight property. This is based on the measured
	 * width of a "M" character multiplied by 1.2, which provides an approximate line height for most fonts.
	 * @method getMeasuredLineHeight
	 * @return {Number} an approximate line height of the text, ignoring the lineHeight property. This is
	 * based on the measured width of a "M" character multiplied by 1.2, which approximates em for most fonts.
	 **/
	p.getMeasuredLineHeight = function() {
		return this._prepContext(Text._workingContext).measureText("M").width*1.2;
	};

	/**
	 * Returns the approximate height of multi-line text by multiplying the number of lines against either the
	 * <code>lineHeight</code> (if specified) or {{#crossLink "Text/getMeasuredLineHeight"}}{{/crossLink}}. Note that
	 * this operation requires the text flowing logic to run, which has an associated CPU cost.
	 * @method getMeasuredHeight
	 * @return {Number} The approximate height of the untransformed multi-line text.
	 **/
	p.getMeasuredHeight = function() {
		return this._drawText(null,{}).height;
	};
	
	/**
	 * @property DisplayObject_getBounds
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_getBounds = p.getBounds;

	/**
	 * Docced in superclass.
	 */
	p.getBounds = function() {
		var rect = this.DisplayObject_getBounds();
		if (rect) { return rect; }
		if (this.text == null || this.text == "") { return null; }
		var o = this._drawText(null, {});
		var w = (this.maxWidth && this.maxWidth < o.width) ? this.maxWidth : o.width;
		var x = w * Text.H_OFFSETS[this.textAlign||"left"];
		var lineHeight = this.lineHeight||this.getMeasuredLineHeight();
		var y = lineHeight * Text.V_OFFSETS[this.textBaseline||"top"];
		return this._rectangle.initialize(x, y, w, o.height);
	};

	/**
	 * Returns a clone of the Text instance.
	 * @method clone
	 * @return {Text} a clone of the Text instance.
	 **/
	p.clone = function() {
		var o = new Text(this.text, this.font, this.color);
		this.cloneProps(o);
		return o;
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Text (text="+  (this.text.length > 20 ? this.text.substr(0, 17)+"..." : this.text) +")]";
	};

// private methods:

	/**
	 * @property DisplayObject_cloneProps
	 * @private
	 * @type Function
	 **/
	p.DisplayObject_cloneProps = p.cloneProps;

	/**
	 * @method cloneProps
	 * @param {Text} o
	 * @protected
	 **/
	p.cloneProps = function(o) {
		this.DisplayObject_cloneProps(o);
		o.textAlign = this.textAlign;
		o.textBaseline = this.textBaseline;
		o.maxWidth = this.maxWidth;
		o.outline = this.outline;
		o.lineHeight = this.lineHeight;
		o.lineWidth = this.lineWidth;
	};

	/**
	 * @method _getWorkingContext
	 * @param {CanvasRenderingContext2D} ctx
	 * @return {CanvasRenderingContext2D}
	 * @protected
	 **/
	p._prepContext = function(ctx) {
		ctx.font = this.font;
		ctx.textAlign = this.textAlign||"left";
		ctx.textBaseline = this.textBaseline||"top";
		return ctx;
	};

	/**
	 * Draws multiline text.
	 * @method _drawText
	 * @param {CanvasRenderingContext2D} ctx
	 * @param {Object} o
	 * @return {Object}
	 * @protected
	 **/
	p._drawText = function(ctx, o) {
		var paint = !!ctx;
		if (!paint) { ctx = this._prepContext(Text._workingContext); }
		var lineHeight = this.lineHeight||this.getMeasuredLineHeight();
		
		var maxW = 0, count = 0;
		var lines = String(this.text).split(/(?:\r\n|\r|\n)/);
		for (var i=0, l=lines.length; i<l; i++) {
			var str = lines[i];
			var w = null;
			
			if (this.lineWidth != null && (w = ctx.measureText(str).width) > this.lineWidth) {
				// text wrapping:
				var words = str.split(/(\s)/);
				str = words[0];
				w = ctx.measureText(str).width;
				
				for (var j=1, jl=words.length; j<jl; j+=2) {
					// Line needs to wrap:
					var wordW = ctx.measureText(words[j] + words[j+1]).width;
					if (w + wordW > this.lineWidth) {
						if (paint) { this._drawTextLine(ctx, str, count*lineHeight); }
						if (w > maxW) { maxW = w; }
						str = words[j+1];
						w = ctx.measureText(str).width;
						count++;
					} else {
						str += words[j] + words[j+1];
						w += wordW;
					}
				}
			}
			
			if (paint) { this._drawTextLine(ctx, str, count*lineHeight); }
			if (o && w == null) { w = ctx.measureText(str).width; }
			if (w > maxW) { maxW = w; }
			count++;
		}
		
		if (o) {
			o.count = count;
			o.width = maxW;
			o.height = count*lineHeight;
		}
		return o;
	};

	/**
	 * @method _drawTextLine
	 * @param {CanvasRenderingContext2D} ctx
	 * @param {String} text
	 * @param {Number} y
	 * @protected
	 **/
	p._drawTextLine = function(ctx, text, y) {
		// Chrome 17 will fail to draw the text if the last param is included but null, so we feed it a large value instead:
			if (this.outline) { ctx.strokeText(text, 0, y, this.maxWidth||0xFFFF); }
			else { ctx.fillText(text, 0, y, this.maxWidth||0xFFFF); }

	};

createjs.Text = Text;
}());
/*
* BitmapText
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

this.createjs = this.createjs || {};

(function () {
	"use strict";

/**
 * Displays text using bitmap glyphs defined in a sprite sheet. Multi-line text is supported
 * using new line characters, but automatic wrapping is not supported. See the 
 * {{#crossLink "BitmapText/spriteSheet:attribute"}}{{/crossLink}}
 * property for more information on defining glyphs.
 * @class BitmapText
 * @extends DisplayObject
 * @param {String} [text=""] The text to display.
 * @param {SpriteSheet} [spriteSheet=null] The spritesheet that defines the character glyphs.
 * @constructor
 **/
function BitmapText(text, spriteSheet) {
	this.initialize(text, spriteSheet);
}
var p = BitmapText.prototype = new createjs.DisplayObject();

// static properties:

// events:

// public properties:
	/**
	 * The text to display.
	 * @property text
	 * @type String
	 * @default ""
	 **/
	p.text = "";
	
	/**
	 * A SpriteSheet instance that defines the glyphs for this bitmap text. Each glyph/character
	 * should have a single frame animation defined in the sprite sheet named the same as
	 * corresponding character. For example, the following animation definition:
	 *
	 * 		"A": {frames: [0]}
	 *
	 * would indicate that the frame at index 0 of the spritesheet should be drawn for the "A" character. The short form
	 * is also acceptable:
	 * 
	 * 		"A": 0
	 *
	 * Note that if a character in the text is not found in the sprite sheet, it will also
	 * try to use the alternate case (upper or lower).
	 *
	 * See SpriteSheet for more information on defining sprite sheet data.
	 * @property spriteSheet
	 * @type String
	 * @default null
	 **/
	p.spriteSheet = null;

	/**
	 * The height of each line of text. If 0, then it will use a line height calculated
	 * by checking for the height of the "1", "T", or "L" character (in that order). If
	 * those characters are not defined, it will use the height of the first frame of the
	 * sprite sheet.
	 * @property lineHeight
	 * @type Number
	 * @default 0
	 **/
	p.lineHeight = 0;

	/**
	 * This spacing (in pixels) will be added after each character in the output.
	 * @property letterSpacing
	 * @type Number
	 * @default 0
	 **/
	p.letterSpacing = 0;

	/**
	 * If a space character is not defined in the sprite sheet, then empty pixels equal to
	 * spaceWidth will be inserted instead. If  0, then it will use a value calculated
	 * by checking for the width of the "1", "E", or "A" character (in that order). If
	 * those characters are not defined, it will use the width of the first frame of the
	 * sprite sheet.
	 * @property spaceWidth
	 * @type Number
	 * @default 0
	 **/
	p.spaceWidth = 0;
	
// private properties:
	
// constructor:
	/**
	 * @property DisplayObject_initialize
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_initialize = p.initialize;
	
	/**
	 * Initialization method.
	 * @method initialize
	 * @param {String} [text=""] The text to display.
	 * @param {SpriteSheet} [spriteSheet=null] The spritesheet that defines the character glyphs.
	 * @protected
	 **/
	p.initialize = function (text, spriteSheet) {
		this.DisplayObject_initialize();

		this.text = text;
		this.spriteSheet = spriteSheet;
	};
	
// public methods:
	/**
	 * @property DisplayObject_draw
	 * @type Function
	 * @protected
	 **/
	p.DisplayObject_draw = p.draw;
	
	/**
	 * Draws the display object into the specified context ignoring it's visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} ignoreCache Indicates whether the draw operation should ignore any current cache.
	 * For example, used for drawing the cache (to prevent it from simply drawing an existing cache back
	 * into itself).
	 **/
	p.draw = function(ctx, ignoreCache) {
		if (this.DisplayObject_draw(ctx, ignoreCache)) { return true; }
		this._drawText(ctx);
	};
	
	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the display object would be visible if drawn to a canvas
	 **/
	p.isVisible = function() {
		var hasContent = this.cacheCanvas || (this.spriteSheet && this.spriteSheet.complete && this.text);
		return !!(this.visible && this.alpha > 0 && this.scaleX != 0 && this.scaleY != 0 && hasContent);
	};
	
	/**
	 * Docced in superclass.
	 */
	p.getBounds = function() {
		var bounds = this._rectangle;
		this._drawText(null, bounds);
		return bounds.width ? bounds : null;
	};

// private methods:
	/**
	 * @method _getFrame
	 * @param {String} character
	 * @param {SpriteSheet} spriteSheet
	 * @protected
	 **/
	p._getFrame = function(character, spriteSheet) {
		var c, o = spriteSheet.getAnimation(character);
		if (!o) {
			(character != (c = character.toUpperCase())) || (character != (c = character.toLowerCase())) || (c=null);
			if (c) { o = spriteSheet.getAnimation(c); }
		}
		return o && spriteSheet.getFrame(o.frames[0]);
	};
	
	/**
	 * @method _getLineHeight
	 * @param {SpriteSheet} ss
	 * @protected
	 **/
	p._getLineHeight = function(ss) {
		var frame = this._getFrame("1",ss) || this._getFrame("T",ss) || this._getFrame("L",ss) || ss.getFrame(0);
		return frame ? frame.rect.height : 1;
	};
	
	/**
	 * @method _getSpaceWidth
	 * @param {SpriteSheet} ss
	 * @protected
	 **/
	p._getSpaceWidth = function(ss) {
		var frame = this._getFrame("1",ss) || this._getFrame("l",ss) || this._getFrame("e",ss) || this._getFrame("a",ss) || ss.getFrame(0);
		return frame ? frame.rect.width : 1;
	};
	
	/**
	 * @method _drawText
	 * @param {CanvasRenderingContext2D} ctx
	 * @param {Object | Rectangle} bounds
	 * @protected
	 **/
	p._drawText = function(ctx, bounds) {
		var w, h, rx, x=0, y=0, spaceW=this.spaceWidth, lineH=this.lineHeight, ss=this.spriteSheet;
				
		var hasSpace = !!this._getFrame(" ", ss);
		if (!hasSpace && spaceW==0) { spaceW = this._getSpaceWidth(ss); }
		if (lineH==0) { lineH = this._getLineHeight(ss); }
		
		var maxX = 0;
		for(var i=0, l=this.text.length; i<l; i++) {
			var character = this.text.charAt(i);
			if (!hasSpace && character == " ") {
				x += spaceW;
				continue;
			} else if (character=="\n" || character=="\r") {
				if (character=="\r" && this.text.charAt(i+1) == "\n") { i++; } // crlf
				if (x-rx > maxX) { maxX = x-rx; }
				x = 0;
				y += lineH;
				continue;
			}

			var o = this._getFrame(character, ss);
			if (!o) { continue; }
			var rect = o.rect;
			rx = o.regX;
			w = rect.width;
			ctx&&ctx.drawImage(o.image, rect.x, rect.y, w, h=rect.height, x-rx, y-o.regY, w, h);
			
			x += w + this.letterSpacing;
		}
		if (x-rx > maxX) { maxX = x-rx; }
		
		if (bounds) {
			bounds.width = maxX-this.letterSpacing;
			bounds.height = y+lineH;
		}
	};

	createjs.BitmapText = BitmapText;
}());/*
* SpriteSheetUtils
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";
// constructor:
/**
 * The SpriteSheetUtils class is a collection of static methods for working with {{#crossLink "SpriteSheet"}}{{/crossLink}}s.
 * A sprite sheet is a series of images (usually animation frames) combined into a single image on a regular grid. For
 * example, an animation consisting of 8 100x100 images could be combined into a 400x200 sprite sheet (4 frames across
 * by 2 high). The SpriteSheetUtils class uses a static interface and should not be instantiated.
 * @class SpriteSheetUtils
 * @static
 **/
var SpriteSheetUtils = function() {
	throw "SpriteSheetUtils cannot be instantiated";
};

	/**
	 * @property _workingCanvas
	 * @static
	 * @type HTMLCanvasElement | Object
	 * @protected
	*/
	
	/**
	 * @property _workingContext
	 * @static
	 * @type CanvasRenderingContext2D
	 * @protected
	*/
	var canvas = (createjs.createCanvas?createjs.createCanvas():document.createElement("canvas"));
	if (canvas.getContext) {
		SpriteSheetUtils._workingCanvas = canvas;
		SpriteSheetUtils._workingContext = canvas.getContext("2d");
		canvas.width = canvas.height = 1;
	}

	

// public static methods:
	/**
	 * <b>This is an experimental method, and may be buggy. Please report issues.</b><br/><br/>
	 * Extends the existing sprite sheet by flipping the original frames horizontally, vertically, or both,
	 * and adding appropriate animation & frame data. The flipped animations will have a suffix added to their names
	 * (_h, _v, _hv as appropriate). Make sure the sprite sheet images are fully loaded before using this method.
	 * <br/><br/>
	 * For example:<br/>
	 * SpriteSheetUtils.addFlippedFrames(mySpriteSheet, true, true);
	 * The above would add frames that are flipped horizontally AND frames that are flipped vertically.
	 * <br/><br/>
	 * Note that you can also flip any display object by setting its scaleX or scaleY to a negative value. On some
	 * browsers (especially those without hardware accelerated canvas) this can result in slightly degraded performance,
	 * which is why addFlippedFrames is available.
	 * @method addFlippedFrames
	 * @static
	 * @param {SpriteSheet} spriteSheet
	 * @param {Boolean} horizontal If true, horizontally flipped frames will be added.
	 * @param {Boolean} vertical If true, vertically flipped frames will be added.
	 * @param {Boolean} both If true, frames that are flipped both horizontally and vertically will be added.
	 * @deprecated Modern browsers perform better when flipping via a transform (ex. scaleX=-1) rendering this obsolete.
	 **/
	SpriteSheetUtils.addFlippedFrames = function(spriteSheet, horizontal, vertical, both) {
		if (!horizontal && !vertical && !both) { return; }

		var count = 0;
		if (horizontal) { SpriteSheetUtils._flip(spriteSheet,++count,true,false); }
		if (vertical) { SpriteSheetUtils._flip(spriteSheet,++count,false,true); }
		if (both) { SpriteSheetUtils._flip(spriteSheet,++count,true,true); }
	};

	/**
	 * Returns a single frame of the specified sprite sheet as a new PNG image. An example of when this may be useful is
	 * to use a spritesheet frame as the source for a bitmap fill.
	 *
	 * <strong>WARNING:</strong> In almost all cases it is better to display a single frame using a {{#crossLink "Sprite"}}{{/crossLink}}
	 * with a {{#crossLink "Sprite/gotoAndStop"}}{{/crossLink}} call than it is to slice out a frame using this
	 * method and display it with a Bitmap instance. You can also crop an image using the {{#crossLink "Bitmap/sourceRect"}}{{/crossLink}}
	 * property of {{#crossLink "Bitmap"}}{{/crossLink}}.
	 *
	 * The extractFrame method may cause cross-domain warnings since it accesses pixels directly on the canvas.
	 * @method extractFrame
	 * @static
	 * @param {Image} spriteSheet The SpriteSheet instance to extract a frame from.
	 * @param {Number|String} frameOrAnimation The frame number or animation name to extract. If an animation
	 * name is specified, only the first frame of the animation will be extracted.
	 * @return {Image} a single frame of the specified sprite sheet as a new PNG image.
	*/
	SpriteSheetUtils.extractFrame = function(spriteSheet, frameOrAnimation) {
		if (isNaN(frameOrAnimation)) {
			frameOrAnimation = spriteSheet.getAnimation(frameOrAnimation).frames[0];
		}
		var data = spriteSheet.getFrame(frameOrAnimation);
		if (!data) { return null; }
		var r = data.rect;
		var canvas = SpriteSheetUtils._workingCanvas;
		canvas.width = r.width;
		canvas.height = r.height;
		SpriteSheetUtils._workingContext.drawImage(data.image, r.x, r.y, r.width, r.height, 0, 0, r.width, r.height);
		var img = document.createElement("img");
		img.src = canvas.toDataURL("image/png");
		return img;
	};

	/**
	 * Merges the rgb channels of one image with the alpha channel of another. This can be used to combine a compressed
	 * JPEG image containing color data with a PNG32 monochromatic image containing alpha data. With certain types of
	 * images (those with detail that lend itself to JPEG compression) this can provide significant file size savings
	 * versus a single RGBA PNG32. This method is very fast (generally on the order of 1-2 ms to run).
	 * @method mergeAlpha
	 * @static
	 * @param {Image} rbgImage The image (or canvas) containing the RGB channels to use.
	 * @param {Image} alphaImage The image (or canvas) containing the alpha channel to use.
	 * @param {Canvas} canvas Optional. If specified, this canvas will be used and returned. If not, a new canvas will be created.
	 * @return {Canvas} A canvas with the combined image data. This can be used as a source for Bitmap or SpriteSheet.
	 * @deprecated Tools such as ImageAlpha generally provide better results. This will be moved to sandbox in the future.
	*/
	SpriteSheetUtils.mergeAlpha = function(rgbImage, alphaImage, canvas) {
		if (!canvas) { canvas = createjs.createCanvas?createjs.createCanvas():document.createElement("canvas"); }
		canvas.width = Math.max(alphaImage.width, rgbImage.width);
		canvas.height = Math.max(alphaImage.height, rgbImage.height);
		var ctx = canvas.getContext("2d");
		ctx.save();
		ctx.drawImage(rgbImage,0,0);
		ctx.globalCompositeOperation = "destination-in";
		ctx.drawImage(alphaImage,0,0);
		ctx.restore();
		return canvas;
	};


// private static methods:
	SpriteSheetUtils._flip = function(spriteSheet, count, h, v) {
		var imgs = spriteSheet._images;
		var canvas = SpriteSheetUtils._workingCanvas;
		var ctx = SpriteSheetUtils._workingContext;
		var il = imgs.length/count;
		for (var i=0;i<il;i++) {
			var src = imgs[i];
			src.__tmp = i; // a bit hacky, but faster than doing indexOf below.
			ctx.setTransform(1,0,0,1,0,0);
			ctx.clearRect(0,0,canvas.width+1,canvas.height+1);
			canvas.width = src.width;
			canvas.height = src.height;
			ctx.setTransform(h?-1:1, 0, 0, v?-1:1, h?src.width:0, v?src.height:0);
			ctx.drawImage(src,0,0);
			var img = document.createElement("img");
			img.src = canvas.toDataURL("image/png");
			// work around a strange bug in Safari:
			img.width = src.width;
			img.height = src.height;
			imgs.push(img);
		}

		var frames = spriteSheet._frames;
		var fl = frames.length/count;
		for (i=0;i<fl;i++) {
			src = frames[i];
			var rect = src.rect.clone();
			img = imgs[src.image.__tmp+il*count];

			var frame = {image:img,rect:rect,regX:src.regX,regY:src.regY};
			if (h) {
				rect.x = img.width-rect.x-rect.width; // update rect
				frame.regX = rect.width-src.regX; // update registration point
			}
			if (v) {
				rect.y = img.height-rect.y-rect.height;  // update rect
				frame.regY = rect.height-src.regY; // update registration point
			}
			frames.push(frame);
		}

		var sfx = "_"+(h?"h":"")+(v?"v":"");
		var names = spriteSheet._animations;
		var data = spriteSheet._data;
		var al = names.length/count;
		for (i=0;i<al;i++) {
			var name = names[i];
			src = data[name];
			var anim = {name:name+sfx,speed:src.speed,next:src.next,frames:[]};
			if (src.next) { anim.next += sfx; }
			frames = src.frames;
			for (var j=0,l=frames.length;j<l;j++) {
				anim.frames.push(frames[j]+fl*count);
			}
			data[anim.name] = anim;
			names.push(anim.name);
		}
	};


createjs.SpriteSheetUtils = SpriteSheetUtils;
}());
/*
* SpriteSheetBuilder
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * The SpriteSheetBuilder allows you to generate sprite sheets at run time from any display object. This can allow
 * you to maintain your assets as vector graphics (for low file size), and render them at run time as sprite sheets
 * for better performance.
 *
 * Sprite sheets can be built either synchronously, or asynchronously, so that large sprite sheets can be generated
 * without locking the UI.
 *
 * Note that the "images" used in the generated sprite sheet are actually canvas elements, and that they will be sized
 * to the nearest power of 2 up to the value of <code>maxWidth</code> or <code>maxHeight</code>.
 * @class SpriteSheetBuilder
 * @extends EventDispatcher
 * @constructor
 **/
var SpriteSheetBuilder = function() {
  this.initialize();
};
var p = SpriteSheetBuilder.prototype = new createjs.EventDispatcher;

// constants:
	SpriteSheetBuilder.ERR_DIMENSIONS = "frame dimensions exceed max spritesheet dimensions";
	SpriteSheetBuilder.ERR_RUNNING = "a build is already running";

// events:

	/**
	 * Dispatched when a build completes.
	 * @event complete
	 * @param {Object} target The object that dispatched the event.
	 * @param {String} type The event type.
	 * @since 0.6.0
	 */

	/**
	 * Dispatched when an asynchronous build has progress.
	 * @event complete
	 * @param {Object} target The object that dispatched the event.
	 * @param {String} type The event type.
	 * @param {Number} progress The current progress value (0-1).
	 * @since 0.6.0
	 */

// public properties:

	/**
	 * The maximum width for the images (not individual frames) in the generated sprite sheet. It is recommended to use
	 * a power of 2 for this value (ex. 1024, 2048, 4096). If the frames cannot all fit within the max dimensions, then
	 * additional images will be created as needed.
	 * @property maxWidth
	 * @type Number
	 * @default 2048
	*/
	p.maxWidth = 2048;

	/**
	 * The maximum height for the images (not individual frames) in the generated sprite sheet. It is recommended to use
	 * a power of 2 for this value (ex. 1024, 2048, 4096). If the frames cannot all fit within the max dimensions, then
	 * additional images will be created as needed.
	 * @property maxHeight
	 * @type Number
	 * @default 2048
	 **/
	p.maxHeight = 2048;

	/**
	 * The sprite sheet that was generated. This will be null before a build is completed successfully.
	 * @property spriteSheet
	 * @type SpriteSheet
	 **/
	p.spriteSheet = null;

	/**
	 * The scale to apply when drawing all frames to the sprite sheet. This is multiplied against any scale specified
	 * in the addFrame call. This can be used, for example, to generate a sprite sheet at run time that is tailored to
	 * the a specific device resolution (ex. tablet vs mobile).
	 * @property scale
	 * @type Number
	 * @default 1
	 **/
	p.scale = 1;

	/**
	* The padding to use between frames. This is helpful to preserve antialiasing on drawn vector content.
	* @property padding
	* @type Number
	* @default 1
	**/
	p.padding = 1;

	/**
	 * A number from 0.01 to 0.99 that indicates what percentage of time the builder can use. This can be
	 * thought of as the number of seconds per second the builder will use. For example, with a timeSlice value of 0.3,
	 * the builder will run 20 times per second, using approximately 15ms per build (30% of available time, or 0.3s per second).
	 * Defaults to 0.3.
	 * @property timeSlice
	 * @type Number
	 * @default 0.3
	 **/
	p.timeSlice = 0.3;

	/**
	 * A value between 0 and 1 that indicates the progress of a build, or -1 if a build has not
	 * been initiated.
	 * @property progress
	 * @type Number
	 * @default -1
	 * @readonly
	 **/
	p.progress = -1;

	// TODO: deprecated.
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "SpriteSheetBuilder/complete:event"}}{{/crossLink}}
	 * event.
	 * @property onComplete
	 * @type Function
	 * @deprecated Use addEventListener and the "complete" event.
	 */
	/**
	 * REMOVED. Use {{#crossLink "EventDispatcher/addEventListener"}}{{/crossLink}} and the {{#crossLink "SpriteSheetBuilder/progress:event"}}{{/crossLink}}
	 * event.
	 * @property onProgress
	 * @type Function
	 * @deprecated Use addEventListener and the "progress" event.
	 */

// private properties:

	/**
	 * @property _frames
	 * @protected
	 * @type Array
	 **/
	p._frames = null;

	/**
	 * @property _animations
	 * @protected
	 * @type Array
	 **/
	p._animations = null;

	/**
	 * @property _data
	 * @protected
	 * @type Array
	 **/
	p._data = null;

	/**
	 * @property _nextFrameIndex
	 * @protected
	 * @type Number
	 **/
	p._nextFrameIndex = 0;

	/**
	 * @property _index
	 * @protected
	 * @type Number
	 **/
	p._index = 0;

	/**
	 * @property _timerID
	 * @protected
	 * @type Number
	 **/
	p._timerID = null;

	/**
	 * @property _scale
	 * @protected
	 * @type Number
	 **/
	p._scale = 1;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	 **/
	p.initialize = function() {
		this._frames = [];
		this._animations = {};
	};

// public methods:

	/**
	 * Adds a frame to the {{#crossLink "SpriteSheet"}}{{/crossLink}}. Note that the frame will not be drawn until you
	 * call {{#crossLink "SpriteSheetBuilder/build"}}{{/crossLink}} method. The optional setup params allow you to have
	 * a function run immediately before the draw occurs. For example, this allows you to add a single source multiple
	 * times, but manipulate it or its children to change it to generate different frames.
	 *
	 * Note that the source's transformations (x, y, scale, rotate, alpha) will be ignored, except for regX/Y. To apply
	 * transforms to a source object and have them captured in the sprite sheet, simply place it into a {{#crossLink "Container"}}{{/crossLink}}
	 * and pass in the Container as the source.
	 * @method addFrame
	 * @param {DisplayObject} source The source {{#crossLink "DisplayObject"}}{{/crossLink}}  to draw as the frame.
	 * @param {Rectangle} [sourceRect] A {{#crossLink "Rectangle"}}{{/crossLink}} defining the portion of the
	 * source to draw to the frame. If not specified, it will look for a <code>getBounds</code> method, bounds property,
	 * or <code>nominalBounds</code> property on the source to use. If one is not found, the frame will be skipped.
	 * @param {Number} [scale=1] Optional. The scale to draw this frame at. Default is 1.
	 * @param {Function} [setupFunction] Optional. A function to call immediately before drawing this frame.
	 * @param {Array} [setupParams] Parameters to pass to the setup function.
	 * @param {Object} [setupScope] The scope to call the setupFunction in.
	 * @return {Number} The index of the frame that was just added, or null if a sourceRect could not be determined.
	 **/
	p.addFrame = function(source, sourceRect, scale, setupFunction, setupParams, setupScope) {
		if (this._data) { throw SpriteSheetBuilder.ERR_RUNNING; }
		var rect = sourceRect||source.bounds||source.nominalBounds;
		if (!rect&&source.getBounds) { rect = source.getBounds(); }
		if (!rect) { return null; }
		scale = scale||1;
		return this._frames.push({source:source, sourceRect:rect, scale:scale, funct:setupFunction, params:setupParams, scope:setupScope, index:this._frames.length, height:rect.height*scale})-1;
	};

	/**
	 * Adds an animation that will be included in the created sprite sheet.
	 * @method addAnimation
	 * @param {String} name The name for the animation.
	 * @param {Array} frames An array of frame indexes that comprise the animation. Ex. [3,6,5] would describe an animation
	 * that played frame indexes 3, 6, and 5 in that order.
	 * @param {String} [next] Specifies the name of the animation to continue to after this animation ends. You can
	 * also pass false to have the animation stop when it ends. By default it will loop to the start of the same animation.
	 * @param {Number} [frequency] Specifies a frame advance frequency for this animation. For example, a value
	 * of 2 would cause the animation to advance every second tick.
	 **/
	p.addAnimation = function(name, frames, next, frequency) {
		if (this._data) { throw SpriteSheetBuilder.ERR_RUNNING; }
		this._animations[name] = {frames:frames, next:next, frequency:frequency};
	};

	/**
	 * This will take a MovieClip, and add its frames and labels to this builder. Labels will be added as an animation
	 * running from the label index to the next label. For example, if there is a label named "foo" at frame 0 and a label
	 * named "bar" at frame 10, in a MovieClip with 15 frames, it will add an animation named "foo" that runs from frame
	 * index 0 to 9, and an animation named "bar" that runs from frame index 10 to 14.
	 *
	 * Note that this will iterate through the full MovieClip with actionsEnabled set to false, ending on the last frame.
	 * @method addMovieClip
	 * @param {MovieClip} source The source MovieClip to add to the sprite sheet.
	 * @param {Rectangle} [sourceRect] A {{#crossLink "Rectangle"}}{{/crossLink}} defining the portion of the source to
	 * draw to the frame. If not specified, it will look for a <code>getBounds</code> method, <code>frameBounds</code>
	 * Array, <code>bounds</code> property, or <code>nominalBounds</code> property on the source to use. If one is not
	 * found, the MovieClip will be skipped.
	 * @param {Number} [scale=1] The scale to draw the movie clip at.
	 **/
	p.addMovieClip = function(source, sourceRect, scale) {
		if (this._data) { throw SpriteSheetBuilder.ERR_RUNNING; }
		var rects = source.frameBounds;
		var rect = sourceRect||source.bounds||source.nominalBounds;
		if (!rect&&source.getBounds) { rect = source.getBounds(); }
		if (!rect && !rects) { return null; }

		var baseFrameIndex = this._frames.length;
		var duration = source.timeline.duration;
		for (var i=0; i<duration; i++) {
			var r = (rects&&rects[i]) ? rects[i] : rect;
			this.addFrame(source, r, scale, function(frame) {
				var ae = this.actionsEnabled;
				this.actionsEnabled = false;
				this.gotoAndStop(frame);
				this.actionsEnabled = ae;
			}, [i], source);
		}
		var labels = source.timeline._labels;
		var lbls = [];
		for (var n in labels) {
			lbls.push({index:labels[n], label:n});
		}
		if (lbls.length) {
			lbls.sort(function(a,b){ return a.index-b.index; });
			for (var i=0,l=lbls.length; i<l; i++) {
				var label = lbls[i].label;
				var start = baseFrameIndex+lbls[i].index;
				var end = baseFrameIndex+((i == l-1) ? duration : lbls[i+1].index);
				var frames = [];
				for (var j=start; j<end; j++) { frames.push(j); }
				this.addAnimation(label, frames, true); // for now, this loops all animations.
			}
		}
	};

	/**
	 * Builds a SpriteSheet instance based on the current frames.
	 * @method build
	 * @return {SpriteSheet} The created SpriteSheet instance, or null if a build is already running or an error occurred.
	 **/
	p.build = function() {
		if (this._data) { throw SpriteSheetBuilder.ERR_RUNNING; }
		this._startBuild();
		while (this._drawNext()) {}
		this._endBuild();
		return this.spriteSheet;
	};

	/**
	 * Asynchronously builds a {{#crossLink "SpriteSheet"}}{{/crossLink}} instance based on the current frames. It will
	 * run 20 times per second, using an amount of time defined by <code>timeSlice</code>. When it is complete it will
	 * call the specified callback.
	 * @method buildAsync
	 * @param {Number} [timeSlice] Sets the timeSlice property on this instance.
	 **/
	p.buildAsync = function(timeSlice) {
		if (this._data) { throw SpriteSheetBuilder.ERR_RUNNING; }
		this.timeSlice = timeSlice;
		this._startBuild();
		var _this = this;
		this._timerID = setTimeout(function() { _this._run(); }, 50-Math.max(0.01, Math.min(0.99, this.timeSlice||0.3))*50);
	};

	/**
	 * Stops the current asynchronous build.
	 * @method stopAsync
	 **/
	p.stopAsync = function() {
		clearTimeout(this._timerID);
		this._data = null;
	};

	/**
	 * SpriteSheetBuilder instances cannot be cloned.
	 * @method clone
	 **/
	p.clone = function() {
		throw("SpriteSheetBuilder cannot be cloned.");
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[SpriteSheetBuilder]";
	};

// private methods:
	/**
	 * @method _startBuild
	 * @protected
	 **/
	p._startBuild = function() {
		var pad = this.padding||0;
		this.progress = 0;
		this.spriteSheet = null;
		this._index = 0;
		this._scale = this.scale;
		var dataFrames = [];
		this._data = {
			images: [],
			frames: dataFrames,
			animations: this._animations // TODO: should we "clone" _animations in case someone adds more animations after a build?
		};

		var frames = this._frames.slice();
		frames.sort(function(a,b) { return (a.height<=b.height) ? -1 : 1; });

		if (frames[frames.length-1].height+pad*2 > this.maxHeight) { throw SpriteSheetBuilder.ERR_DIMENSIONS; }
		var y=0, x=0;
		var img = 0;
		while (frames.length) {
			var o = this._fillRow(frames, y, img, dataFrames, pad);
			if (o.w > x) { x = o.w; }
			y += o.h;
			if (!o.h || !frames.length) {
				var canvas = createjs.createCanvas?createjs.createCanvas():document.createElement("canvas");
				canvas.width = this._getSize(x,this.maxWidth);
				canvas.height = this._getSize(y,this.maxHeight);
				this._data.images[img] = canvas;
				if (!o.h) {
					x=y=0;
					img++;
				}
			}
		}
	};

	/**
	 * @method _getSize
	 * @protected
	 * @return {Number} The width & height of the row.
	 **/
	p._getSize = function(size,max) {
		var pow = 4;
		while (Math.pow(2,++pow) < size){}
		return Math.min(max,Math.pow(2,pow));
	};

	/**
	 * @method _fillRow
	 * @param {Array} frames
	 * @param {Number} y
	 * @param {Image} img
	 * @param {Object} dataFrames
	 * @param {Number} pad
	 * @protected
	 * @return {Number} The width & height of the row.
	 **/
	p._fillRow = function(frames, y, img, dataFrames, pad) {
		var w = this.maxWidth;
		var maxH = this.maxHeight;
		y += pad;
		var h = maxH-y;
		var x = pad;
		var height = 0;
		for (var i=frames.length-1; i>=0; i--) {
			var frame = frames[i];
			var sc = this._scale*frame.scale;
			var rect = frame.sourceRect;
			var source = frame.source;
			var rx = Math.floor(sc*rect.x-pad);
			var ry = Math.floor(sc*rect.y-pad);
			var rh = Math.ceil(sc*rect.height+pad*2);
			var rw = Math.ceil(sc*rect.width+pad*2);
			if (rw > w) { throw SpriteSheetBuilder.ERR_DIMENSIONS; }
			if (rh > h || x+rw > w) { continue; }
			frame.img = img;
			frame.rect = new createjs.Rectangle(x,y,rw,rh);
			height = height || rh;
			frames.splice(i,1);
			dataFrames[frame.index] = [x,y,rw,rh,img,Math.round(-rx+sc*source.regX-pad),Math.round(-ry+sc*source.regY-pad)];
			x += rw;
		}
		return {w:x, h:height};
	};

	/**
	 * @method _endBuild
	 * @protected
	 **/
	p._endBuild = function() {
		this.spriteSheet = new createjs.SpriteSheet(this._data);
		this._data = null;
		this.progress = 1;
		this.dispatchEvent("complete");
	};

	/**
	 * @method _run
	 * @protected
	 **/
	p._run = function() {
		var ts = Math.max(0.01, Math.min(0.99, this.timeSlice||0.3))*50;
		var t = (new Date()).getTime()+ts;
		var complete = false;
		while (t > (new Date()).getTime()) {
			if (!this._drawNext()) { complete = true; break; }
		}
		if (complete) {
			this._endBuild();
		} else {
			var _this = this;
			this._timerID = setTimeout(function() { _this._run(); }, 50-ts);
		}
		var p = this.progress = this._index/this._frames.length;
		if (this.hasEventListener("progress")) {
			var evt = new createjs.Event("progress");
			evt.progress = p;
			this.dispatchEvent(evt);
		}
	};

	/**
	 * @method _drawNext
	 * @protected
	 * @return Boolean Returns false if this is the last draw.
	 **/
	p._drawNext = function() {
		var frame = this._frames[this._index];
		var sc = frame.scale*this._scale;
		var rect = frame.rect;
		var sourceRect = frame.sourceRect;
		var canvas = this._data.images[frame.img];
		var ctx = canvas.getContext("2d");
		frame.funct&&frame.funct.apply(frame.scope, frame.params);
		ctx.save();
		ctx.beginPath();
		ctx.rect(rect.x, rect.y, rect.width, rect.height);
		ctx.clip();
		ctx.translate(Math.ceil(rect.x-sourceRect.x*sc), Math.ceil(rect.y-sourceRect.y*sc));
		ctx.scale(sc,sc);
		frame.source.draw(ctx); // display object will draw itself.
		ctx.restore();
		return (++this._index) < this._frames.length;
	};

createjs.SpriteSheetBuilder = SpriteSheetBuilder;
}());
/*
* DOMElement
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";
// TODO: fix problems with rotation.
// TODO: exclude from getObjectsUnderPoint

/**
 * <b>This class is still experimental, and more advanced use is likely to be buggy. Please report bugs.</b>
 *
 * A DOMElement allows you to associate a HTMLElement with the display list. It will be transformed
 * within the DOM as though it is child of the {{#crossLink "Container"}}{{/crossLink}} it is added to. However, it is
 * not rendered to canvas, and as such will retain whatever z-index it has relative to the canvas (ie. it will be
 * drawn in front of or behind the canvas).
 *
 * The position of a DOMElement is relative to their parent node in the DOM. It is recommended that
 * the DOM Object be added to a div that also contains the canvas so that they share the same position
 * on the page.
 *
 * DOMElement is useful for positioning HTML elements over top of canvas content, and for elements
 * that you want to display outside the bounds of the canvas. For example, a tooltip with rich HTML
 * content.
 *
 * <h4>Mouse Interaction</h4>
 *
 * DOMElement instances are not full EaselJS display objects, and do not participate in EaselJS mouse
 * events or support methods like hitTest. To get mouse events from a DOMElement, you must instead add handlers to
 * the htmlElement (note, this does not support EventDispatcher)
 *
 *      var domElement = new createjs.DOMElement(htmlElement);
 *      domElement.htmlElement.onclick = function() {
 *          console.log("clicked");
 *      }
 *
 * @class DOMElement
 * @extends DisplayObject
 * @constructor
 * @param {HTMLElement} htmlElement A reference or id for the DOM element to manage.
 */
var DOMElement = function(htmlElement) {
  this.initialize(htmlElement);
};
var p = DOMElement.prototype = new createjs.DisplayObject();

// public properties:
	/**
	 * The DOM object to manage.
	 * @property htmlElement
	 * @type HTMLElement
	 */
	p.htmlElement = null;

// private properties:
	/**
	 * @property _oldMtx
	 * @type Matrix2D
	 * @protected
	 */
	p._oldMtx = null;
	
	/**
	 * @property _visible
	 * @type Boolean
	 * @protected
	 */
	p._visible = false;

// constructor:
	/**
	 * @property DisplayObject_initialize
	 * @type Function
	 * @private
	 */
	p.DisplayObject_initialize = p.initialize;

	/**
	 * Initialization method.
	 * @method initialize
	 * @param {HTMLElement} htmlElement A reference or id for the DOM element to manage.
	 * @protected
	*/
	p.initialize = function(htmlElement) {
		if (typeof(htmlElement)=="string") { htmlElement = document.getElementById(htmlElement); }
		this.DisplayObject_initialize();
		this.mouseEnabled = false;
		this.htmlElement = htmlElement;
		var style = htmlElement.style;
		// this relies on the _tick method because draw isn't called if a parent is not visible.
		style.position = "absolute";
		style.transformOrigin = style.WebkitTransformOrigin = style.msTransformOrigin = style.MozTransformOrigin = style.OTransformOrigin = "0% 0%";
	};

// public methods:
	/**
	 * Returns true or false indicating whether the display object would be visible if drawn to a canvas.
	 * This does not account for whether it would be visible within the boundaries of the stage.
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method isVisible
	 * @return {Boolean} Boolean indicating whether the display object would be visible if drawn to a canvas
	 */
	p.isVisible = function() {
		return this.htmlElement != null;
	};

	/**
	 * Draws the display object into the specified context ignoring its visible, alpha, shadow, and transform.
	 * Returns true if the draw was handled (useful for overriding functionality).
	 * NOTE: This method is mainly for internal use, though it may be useful for advanced uses.
	 * @method draw
	 * @param {CanvasRenderingContext2D} ctx The canvas 2D context object to draw into.
	 * @param {Boolean} ignoreCache Indicates whether the draw operation should ignore any current cache.
	 * For example, used for drawing the cache (to prevent it from simply drawing an existing cache back
	 * into itself).
	 * @return {Boolean}
	 */
	p.draw = function(ctx, ignoreCache) {
		// this relies on the _tick method because draw isn't called if a parent is not visible.
		if (this.visible) { this._visible = true; }
		// the actual update happens in _handleDrawEnd
		return true;
	};

	/**
	 * Not applicable to DOMElement.
	 * @method cache
	 */
	p.cache = function() {};

	/**
	 * Not applicable to DOMElement.
	 * @method uncache
	 */
	p.uncache = function() {};

	/**
	 * Not applicable to DOMElement.
	 * @method updateCache
	 */
	p.updateCache = function() {};

	/**
	 * Not applicable to DOMElement.
	 * @method hitTest
	 */
	p.hitTest = function() {};

	/**
	 * Not applicable to DOMElement.
	 * @method localToGlobal
	 */
	p.localToGlobal = function() {};

	/**
	 * Not applicable to DOMElement.
	 * @method globalToLocal
	 */
	p.globalToLocal = function() {};

	/**
	 * Not applicable to DOMElement.
	 * @method localToLocal
	 */
	p.localToLocal = function() {};

	/**
	 * DOMElement cannot be cloned. Throws an error.
	 * @method clone
	 */
	p.clone = function() {
		throw("DOMElement cannot be cloned.")
	};

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 */
	p.toString = function() {
		return "[DOMElement (name="+  this.name +")]";
	};

	/**
     * Interaction events should be added to `htmlElement`, and not the DOMElement instance, since DOMElement instances
	 * are not full EaselJS display objects and do not participate in EaselJS mouse events.
	 * @event click
	 */

     /**
     * Interaction events should be added to `htmlElement`, and not the DOMElement instance, since DOMElement instances
 	 * are not full EaselJS display objects and do not participate in EaselJS mouse events.
	 * @event dblClick
	 */

     /**
      * Interaction events should be added to `htmlElement`, and not the DOMElement instance, since DOMElement instances
 	  * are not full EaselJS display objects and do not participate in EaselJS mouse events.
	  * @event mousedown
	  */

     /**
      * The HTMLElement can listen for the mouseover event, not the DOMElement instance.
      * Since DOMElement instances are not full EaselJS display objects and do not participate in EaselJS mouse events.
      * @event mouseover
	  */

     /**
      * Not applicable to DOMElement.
	  * @event tick
	  */


// private methods:
	/**
	 * @property DisplayObject__tick
	 * @type Function
	 * @protected
	 */
	p.DisplayObject__tick = p._tick;

	/**
	 * @method _tick
	 * @param {Array} params Parameters to pass onto the DisplayObject {{#crossLink "DisplayObject/tick"}}{{/crossLink}}
	 * function.
	 * @protected
	 */
	p._tick = function(params) {
		var stage = this.getStage();
		this._visible = false;
		stage&&stage.on("drawend", this._handleDrawEnd, this, true);
		this.DisplayObject__tick(params);
	};
	
	/**
	 * @method _handleDrawEnd
	 * @param {Event} evt
	 * @protected
	 */
	p._handleDrawEnd = function(evt) {
		var o = this.htmlElement;
		if (!o) { return; }
		var style = o.style;
		
		var visibility = this._visible ? "visible" : "hidden";
		if (visibility != style.visibility) { style.visibility = visibility; }
		if (!this._visible) { return; }
		
		var mtx = this.getConcatenatedMatrix(this._matrix);
		var oMtx = this._oldMtx;
		var n = 10000; // precision
		if (!oMtx || oMtx.alpha != mtx.alpha) {
			style.opacity = ""+(mtx.alpha*n|0)/n;
			if (oMtx) { oMtx.alpha = mtx.alpha; }
		}
		if (!oMtx || oMtx.tx != mtx.tx || oMtx.ty != mtx.ty || oMtx.a != mtx.a || oMtx.b != mtx.b || oMtx.c != mtx.c || oMtx.d != mtx.d) {
			var str = "matrix(" + (mtx.a*n|0)/n +","+ (mtx.b*n|0)/n +","+ (mtx.c*n|0)/n +","+ (mtx.d*n|0)/n +","+ (mtx.tx+0.5|0);
			style.transform = style.WebkitTransform = style.OTransform = style.msTransform = str +","+ (mtx.ty+0.5|0) +")";
			style.MozTransform = str +"px,"+ (mtx.ty+0.5|0) +"px)";
			this._oldMtx = oMtx ? oMtx.copy(mtx) : mtx.clone();
		}
	};

createjs.DOMElement = DOMElement;
}());
/*
* Filter
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Base class that all filters should inherit from. Filters need to be applied to objects that have been cached using
 * the {{#crossLink "DisplayObject/cache"}}{{/crossLink}} method. If an object changes, please cache it again, or use
 * {{#crossLink "DisplayObject/updateCache"}}{{/crossLink}}. Note that the filters must be applied before caching.
 *
 * <h4>Example</h4>
 *      myInstance.filters = [
 *          new createjs.ColorFilter(0, 0, 0, 1, 255, 0, 0),
 *          new createjs.BlurFilter(5, 5, 10)
 *      ];
 *      myInstance.cache(0,0, 100, 100);
 *
 * Note that each filter can implement a {{#crossLink "Filter/getBounds"}}{{/crossLink}} method, which returns the
 * margins that need to be applied in order to fully display the filter. For example, the {{#crossLink "BlurFilter"}}{{/crossLink}}
 * will cause an object to feather outwards, resulting in a margin around the shape.
 *
 * <h4>EaselJS Filters</h4>
 * EaselJS comes with a number of pre-built filters. Note that individual filters are not compiled into the minified
 * version of EaselJS. To use them, you must include them manually in the HTML.
 * <ul><li>{{#crossLink "AlphaMapFilter"}}{{/crossLink}} : Map a greyscale image to the alpha channel of a display object</li>
 *      <li>{{#crossLink "AlphaMaskFilter"}}{{/crossLink}}: Map an image's alpha channel to the alpha channel of a display object</li>
 *      <li>{{#crossLink "BlurFilter"}}{{/crossLink}}: Apply vertical and horizontal blur to a display object</li>
 *      <li>{{#crossLink "ColorFilter"}}{{/crossLink}}: Color transform a display object</li>
 *      <li>{{#crossLink "ColorMatrixFilter"}}{{/crossLink}}: Transform an image using a {{#crossLink "ColorMatrix"}}{{/crossLink}}</li>
 * </ul>
 *
 * @class Filter
 * @constructor
 **/
var Filter = function() {
  this.initialize();
};
var p = Filter.prototype;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @protected
	 **/
	p.initialize = function() {}

// public methods:
	/**
	 * Returns a rectangle with values indicating the margins required to draw the filter or null.
	 * For example, a filter that will extend the drawing area 4 pixels to the left, and 7 pixels to the right
	 * (but no pixels up or down) would return a rectangle with (x=-4, y=0, width=11, height=0).
	 * @method getBounds
	 * @return {Rectangle} a rectangle object indicating the margins required to draw the filter or null if the filter does not effect bounds.
	 **/
	p.getBounds = function() {
		return null;
	};

	/**
	 * Applies the filter to the specified context.
	 * @method applyFilter
	 * @param {CanvasRenderingContext2D} ctx The 2D context to use as the source.
	 * @param {Number} x The x position to use for the source rect.
	 * @param {Number} y The y position to use for the source rect.
	 * @param {Number} width The width to use for the source rect.
	 * @param {Number} height The height to use for the source rect.
	 * @param {CanvasRenderingContext2D} [targetCtx] The 2D context to draw the result to. Defaults to the context passed to ctx.
	 * @param {Number} [targetX] The x position to draw the result to. Defaults to the value passed to x.
	 * @param {Number} [targetY] The y position to draw the result to. Defaults to the value passed to y.
	 * @return {Boolean} If the filter was applied successfully.
	 **/
	p.applyFilter = function(ctx, x, y, width, height, targetCtx, targetX, targetY) {}

	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[Filter]";
	};

	/**
	 * Returns a clone of this Filter instance.
	 * @method clone
	 * @return {Filter} A clone of the current Filter instance.
	 **/
	p.clone = function() {
		return new Filter();
	};

createjs.Filter = Filter;
}());
/*
* BlurFilter
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Applies a box blur to DisplayObjects. Note that this filter is fairly CPU intensive, particularly if the quality is
 * set higher than 1.
 *
 * <h4>Example</h4>
 * This example creates a red circle, and then applies a 5 pixel blur to it. It uses the {{#crossLink "Filter/getBounds"}}{{/crossLink}}
 * method to account for the spread that the blur causes.
 *
 *      var shape = new createjs.Shape().set({x:100,y:100});
 *      shape.graphics.beginFill("#ff0000").drawCircle(0,0,50);
 *
 *      var blurFilter = new createjs.BlurFilter(5, 5, 1);
 *      shape.filters = [blurFilter];
 *      var bounds = blurFilter.getBounds();
 *
 *      shape.cache(-50+bounds.x, -50+bounds.y, 100+bounds.width, 100+bounds.height);
 *
 * See {{#crossLink "Filter"}}{{/crossLink}} for an more information on applying filters.
 * @class BlurFilter
 * @extends Filter
 * @constructor
 * @param {Number} [blurX=0] The horizontal blur radius in pixels.
 * @param {Number} [blurY=0] The vertical blur radius in pixels.
 * @param {Number} [quality=1] The number of blur iterations.
 **/
var BlurFilter = function( blurX, blurY, quality ) {
  this.initialize( blurX, blurY, quality );
};
var p = BlurFilter.prototype = new createjs.Filter();

// constructor:
	/** @ignore */
	p.initialize = function( blurX, blurY, quality ) {
		if ( isNaN(blurX) || blurX < 0 ) blurX = 0;
		this.blurX = blurX | 0;
		if ( isNaN(blurY) || blurY < 0 ) blurY = 0;
		this.blurY = blurY | 0;
		if ( isNaN(quality) || quality < 1  ) quality = 1;
		this.quality = quality | 0;
	};

// public properties:

	/**
	 * Horizontal blur radius in pixels
	 * @property blurX
	 * @default 0
	 * @type Number
	 **/
	p.blurX = 0;

	/**
	 * Vertical blur radius in pixels
	 * @property blurY
	 * @default 0
	 * @type Number
	 **/
	p.blurY = 0;

	/**
	 * Number of blur iterations. For example, a value of 1 will produce a rough blur. A value of 2 will produce a
	 * smoother blur, but take twice as long to run.
	 * @property quality
	 * @default 1
	 * @type Number
	 **/
	p.quality = 1;
	
	//TODO: There might be a better better way to place these two lookup tables:
	p.mul_table = [ 1,171,205,293,57,373,79,137,241,27,391,357,41,19,283,265,497,469,443,421,25,191,365,349,335,161,155,149,9,278,269,261,505,245,475,231,449,437,213,415,405,395,193,377,369,361,353,345,169,331,325,319,313,307,301,37,145,285,281,69,271,267,263,259,509,501,493,243,479,118,465,459,113,446,55,435,429,423,209,413,51,403,199,393,97,3,379,375,371,367,363,359,355,351,347,43,85,337,333,165,327,323,5,317,157,311,77,305,303,75,297,294,73,289,287,71,141,279,277,275,68,135,67,133,33,262,260,129,511,507,503,499,495,491,61,121,481,477,237,235,467,232,115,457,227,451,7,445,221,439,218,433,215,427,425,211,419,417,207,411,409,203,202,401,399,396,197,49,389,387,385,383,95,189,47,187,93,185,23,183,91,181,45,179,89,177,11,175,87,173,345,343,341,339,337,21,167,83,331,329,327,163,81,323,321,319,159,79,315,313,39,155,309,307,153,305,303,151,75,299,149,37,295,147,73,291,145,289,287,143,285,71,141,281,35,279,139,69,275,137,273,17,271,135,269,267,133,265,33,263,131,261,130,259,129,257,1];
        
   
	p.shg_table = [0,9,10,11,9,12,10,11,12,9,13,13,10,9,13,13,14,14,14,14,10,13,14,14,14,13,13,13,9,14,14,14,15,14,15,14,15,15,14,15,15,15,14,15,15,15,15,15,14,15,15,15,15,15,15,12,14,15,15,13,15,15,15,15,16,16,16,15,16,14,16,16,14,16,13,16,16,16,15,16,13,16,15,16,14,9,16,16,16,16,16,16,16,16,16,13,14,16,16,15,16,16,10,16,15,16,14,16,16,14,16,16,14,16,16,14,15,16,16,16,14,15,14,15,13,16,16,15,17,17,17,17,17,17,14,15,17,17,16,16,17,16,15,17,16,17,11,17,16,17,16,17,16,17,17,16,17,17,16,17,17,16,16,17,17,17,16,14,17,17,17,17,15,16,14,16,15,16,13,16,15,16,14,16,15,16,12,16,15,16,17,17,17,17,17,13,16,15,17,17,17,16,15,17,17,17,16,15,17,17,14,16,17,17,16,17,17,16,15,17,16,14,17,16,15,17,16,17,17,16,17,15,16,17,14,17,16,15,17,16,17,13,17,16,17,17,16,17,14,17,16,17,16,17,16,17,9];

// public methods:
	/** docced in super class **/
	p.getBounds = function() {
		var q = Math.pow(this.quality, 0.6)*0.5;
		return new createjs.Rectangle(-this.blurX*q,-this.blurY*q,2*this.blurX*q,2*this.blurY*q);
	};

	p.applyFilter = function(ctx, x, y, width, height, targetCtx, targetX, targetY) {
		targetCtx = targetCtx || ctx;
		if (targetX == null) { targetX = x; }
		if (targetY == null) { targetY = y; }
		try {
			var imageData = ctx.getImageData(x, y, width, height);
		} catch(e) {
			//if (!this.suppressCrossDomainErrors) throw new Error("unable to access local image data: " + e);
			return false;
		}

		var radiusX = this.blurX/2;
		if ( isNaN(radiusX) || radiusX < 0 ) return false;
		radiusX |= 0;

		var radiusY = this.blurY/2;
		if ( isNaN(radiusY) || radiusY < 0 ) return false;
		radiusY |= 0;

		if ( radiusX == 0 && radiusY == 0 ) return false;

		var iterations = this.quality;
		if ( isNaN(iterations) || iterations < 1  ) iterations = 1;
		iterations |= 0;
		if ( iterations > 3 ) iterations = 3;
		if ( iterations < 1 ) iterations = 1;

		var pixels = imageData.data;

		var x, y, i, p, yp, yi, yw, r_sum, g_sum, b_sum, a_sum, 
		r_out_sum, g_out_sum, b_out_sum, a_out_sum,
		r_in_sum, g_in_sum, b_in_sum, a_in_sum, 
		pr, pg, pb, pa, rbs;

		var divx = radiusX + radiusX + 1;
		var divy = radiusY + radiusY + 1;
		var w4 = width << 2;
		var widthMinus1  = width - 1;
		var heightMinus1 = height - 1;
		var rxp1  = radiusX + 1;
		var ryp1  = radiusY + 1;
		var stackStartX = {r:0,b:0,g:0,a:0,next:null};
		var stackx = stackStartX;
		for ( i = 1; i < divx; i++ )
		{
			stackx = stackx.next = {r:0,b:0,g:0,a:0,next:null};
			if ( i == rxp1 ) var stackEndX = stackx;
		}
		stackx.next = stackStartX;
		
		var stackStartY = {r:0,b:0,g:0,a:0,next:null};
		var stacky = stackStartY;
		for ( i = 1; i < divy; i++ )
		{
			stacky = stacky.next = {r:0,b:0,g:0,a:0,next:null};
			if ( i == ryp1 ) var stackEndY = stacky;
		}
		stacky.next = stackStartY;
		
		var stackIn = null;



		
		while ( iterations-- > 0 ) {
			yw = yi = 0;
			var mul_sum = this.mul_table[radiusX];
			var shg_sum = this.shg_table[radiusX];
			for ( y = height; --y > -1; )
			{
				r_sum = rxp1 * ( pr = pixels[yi] );
				g_sum = rxp1 * ( pg = pixels[yi+1] );
				b_sum = rxp1 * ( pb = pixels[yi+2] );
				a_sum = rxp1 * ( pa = pixels[yi+3] );

				stackx = stackStartX;

				for( i = rxp1; --i > -1; )
				{
					stackx.r = pr;
					stackx.g = pg;
					stackx.b = pb;
					stackx.a = pa;
					stackx = stackx.next;
				}

				for( i = 1; i < rxp1; i++ )
				{
					p = yi + (( widthMinus1 < i ? widthMinus1 : i ) << 2 );
					r_sum += ( stackx.r = pixels[p]);
					g_sum += ( stackx.g = pixels[p+1]);
					b_sum += ( stackx.b = pixels[p+2]);
					a_sum += ( stackx.a = pixels[p+3]);

					stackx = stackx.next;
				}

				stackIn = stackStartX;
				for ( x = 0; x < width; x++ )
				{
					pixels[yi++] = (r_sum * mul_sum) >>> shg_sum;
					pixels[yi++] = (g_sum * mul_sum) >>> shg_sum;
					pixels[yi++] = (b_sum * mul_sum) >>> shg_sum;
					pixels[yi++] = (a_sum * mul_sum) >>> shg_sum;

					p =  ( yw + ( ( p = x + radiusX + 1 ) < widthMinus1 ? p : widthMinus1 ) ) << 2;

					r_sum -= stackIn.r - ( stackIn.r = pixels[p]);
					g_sum -= stackIn.g - ( stackIn.g = pixels[p+1]);
					b_sum -= stackIn.b - ( stackIn.b = pixels[p+2]);
					a_sum -= stackIn.a - ( stackIn.a = pixels[p+3]);

					stackIn = stackIn.next;

				}
				yw += width;
			}

			mul_sum = this.mul_table[radiusY];
			shg_sum = this.shg_table[radiusY];
			for ( x = 0; x < width; x++ )
			{
				yi = x << 2;

				r_sum = ryp1 * ( pr = pixels[yi]);
				g_sum = ryp1 * ( pg = pixels[yi+1]);
				b_sum = ryp1 * ( pb = pixels[yi+2]);
				a_sum = ryp1 * ( pa = pixels[yi+3]);

				stacky = stackStartY;

				for( i = 0; i < ryp1; i++ )
				{
					stacky.r = pr;
					stacky.g = pg;
					stacky.b = pb;
					stacky.a = pa;
					stacky = stacky.next;
				}

				yp = width;

				for( i = 1; i <= radiusY; i++ )
				{
					yi = ( yp + x ) << 2;

					r_sum += ( stacky.r = pixels[yi]);
					g_sum += ( stacky.g = pixels[yi+1]);
					b_sum += ( stacky.b = pixels[yi+2]);
					a_sum += ( stacky.a = pixels[yi+3]);

					stacky = stacky.next;

					if( i < heightMinus1 )
					{
						yp += width;
					}
				}

				yi = x;
				stackIn = stackStartY;
				if ( iterations > 0 )
				{
					for ( y = 0; y < height; y++ )
					{
						p = yi << 2;
						pixels[p+3] = pa =(a_sum * mul_sum) >>> shg_sum;
						if ( pa > 0 )
						{
							pixels[p]   = ((r_sum * mul_sum) >>> shg_sum ); 
							pixels[p+1] = ((g_sum * mul_sum) >>> shg_sum );
							pixels[p+2] = ((b_sum * mul_sum) >>> shg_sum );
						} else {
							pixels[p] = pixels[p+1] = pixels[p+2] = 0
						}

						p = ( x + (( ( p = y + ryp1) < heightMinus1 ? p : heightMinus1 ) * width )) << 2;

						r_sum -= stackIn.r - ( stackIn.r = pixels[p]);
						g_sum -= stackIn.g - ( stackIn.g = pixels[p+1]);
						b_sum -= stackIn.b - ( stackIn.b = pixels[p+2]);
						a_sum -= stackIn.a - ( stackIn.a = pixels[p+3]);

						stackIn = stackIn.next;

						yi += width;
					}
				} else {
					for ( y = 0; y < height; y++ )
					{
						p = yi << 2;
						pixels[p+3] = pa =(a_sum * mul_sum) >>> shg_sum;
						if ( pa > 0 )
						{
							pa = 255 / pa;
							pixels[p]   = ((r_sum * mul_sum) >>> shg_sum ) * pa; 
							pixels[p+1] = ((g_sum * mul_sum) >>> shg_sum ) * pa;
							pixels[p+2] = ((b_sum * mul_sum) >>> shg_sum ) * pa;
						} else {
							pixels[p] = pixels[p+1] = pixels[p+2] = 0
						}

						p = ( x + (( ( p = y + ryp1) < heightMinus1 ? p : heightMinus1 ) * width )) << 2;

						r_sum -= stackIn.r - ( stackIn.r = pixels[p]);
						g_sum -= stackIn.g - ( stackIn.g = pixels[p+1]);
						b_sum -= stackIn.b - ( stackIn.b = pixels[p+2]);
						a_sum -= stackIn.a - ( stackIn.a = pixels[p+3]);

						stackIn = stackIn.next;

						yi += width;
					}
				}
			}
		}
		targetCtx.putImageData(imageData, targetX, targetY);
		return true;
	};

	/**
	 * Returns a clone of this object.
	 * @method clone
	 * @return {BlurFilter}
	 **/
	p.clone = function() {
		return new BlurFilter(this.blurX, this.blurY, this.quality);
	};

	p.toString = function() {
		return "[BlurFilter]";
	};

	createjs.BlurFilter = BlurFilter;

}());
/*
 * AlphaMapFilter
 * Visit http://createjs.com/ for documentation, updates and examples.
 *
 * Copyright (c) 2010 gskinner.com, inc.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs || {};

(function () {
	"use strict";
	/**
	 * Applies a greyscale alpha map image (or canvas) to the target, such that the alpha channel of the result will
	 * be copied from the red channel of the map, and the RGB channels will be copied from the target.
	 *
	 * Generally, it is recommended that you use {{#crossLink "AlphaMaskFilter"}}{{/crossLink}}, because it has much
	 * better performance.
	 *
	 * <h4>Example</h4>
	 * This example draws a red->blue box, caches it, and then uses the cache canvas as an alpha map on a 100x100 image.
	 *
	 *       var box = new createjs.Shape();
	 *       box.graphics.beginLinearGradientFill(["#ff0000", "#0000ff"], [0, 1], 0, 0, 0, 100)
	 *       box.graphics.drawRect(0, 0, 100, 100);
	 *       box.cache(0, 0, 100, 100);
	 *
	 *       var bmp = new createjs.Bitmap("path/to/image.jpg");
	 *       bmp.filters = [
	 *           new createjs.AlphaMapFilter(box.cacheCanvas)
	 *       ];
	 *       bmp.cache(0, 0, 100, 100);
	 *       stage.addChild(bmp);
	 *
	 * See {{#crossLink "Filter"}}{{/crossLink}} for more information on applying filters.
	 * @class AlphaMapFilter
	 * @extends Filter
	 * @constructor
	 * @param {Image|HTMLCanvasElement} alphaMap The greyscale image (or canvas) to use as the alpha value for the
	 * result. This should be exactly the same dimensions as the target.
	 **/
	var AlphaMapFilter = function (alphaMap) {
		this.initialize(alphaMap);
	};
	var p = AlphaMapFilter.prototype = new createjs.Filter();

// constructor:
	/** @ignore */
	p.initialize = function (alphaMap) {
		this.alphaMap = alphaMap;
	};

// public properties:

	/**
	 * The greyscale image (or canvas) to use as the alpha value for the result. This should be exactly the same
	 * dimensions as the target.
	 * @property alphaMap
	 * @type Image|HTMLCanvasElement
	 **/
	p.alphaMap = null;

// private properties:
	p._alphaMap = null;
	p._mapData = null;

// public methods:

	p.applyFilter = function (ctx, x, y, width, height, targetCtx, targetX, targetY) {
		if (!this.alphaMap) {
			return true;
		}
		if (!this._prepAlphaMap()) {
			return false;
		}
		targetCtx = targetCtx || ctx;
		if (targetX == null) {
			targetX = x;
		}
		if (targetY == null) {
			targetY = y;
		}

		try {
			var imageData = ctx.getImageData(x, y, width, height);
		} catch (e) {
			//if (!this.suppressCrossDomainErrors) throw new Error("unable to access local image data: " + e);
			return false;
		}
		var data = imageData.data;
		var map = this._mapData;
		var l = data.length;
		for(var i = 0; i < l; i += 4) {
			data[i + 3] = map[i] || 0;
		}
		targetCtx.putImageData(imageData, targetX, targetY);
		return true;
	};

	/**
	 * Returns a clone of this object.
	 * @method clone
	 * @return {AlphaMapFilter} A clone of the current AlphaMapFilter instance.
	 **/
	p.clone = function () {
		return new AlphaMapFilter(this.alphaMap);
	};

	p.toString = function () {
		return "[AlphaMapFilter]";
	};

// private methods:
	p._prepAlphaMap = function () {
		if (!this.alphaMap) {
			return false;
		}
		if (this.alphaMap == this._alphaMap && this._mapData) {
			return true;
		}

		this._mapData = null;
		var map = this._alphaMap = this.alphaMap;
		var canvas = map;
		var ctx;
		if (map instanceof HTMLCanvasElement) {
			ctx = canvas.getContext("2d");
		} else {
			canvas = createjs.createCanvas ? createjs.createCanvas() : document.createElement("canvas");
			canvas.width = map.width;
			canvas.height = map.height;
			ctx = canvas.getContext("2d");
			ctx.drawImage(map, 0, 0);
		}

		try {
			var imgData = ctx.getImageData(0, 0, map.width, map.height);
		} catch (e) {
			//if (!this.suppressCrossDomainErrors) throw new Error("unable to access local image data: " + e);
			return false;
		}
		this._mapData = imgData.data;
		return true;
	};

	createjs.AlphaMapFilter = AlphaMapFilter;

}());
/*
 * AlphaMaskFilter
 * Visit http://createjs.com/ for documentation, updates and examples.
 *
 * Copyright (c) 2010 gskinner.com, inc.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs || {};

(function () {
	"use strict";

	/**
	 * Applies the alpha from the mask image (or canvas) to the target, such that the alpha channel of the result will
	 * be derived from the mask, and the RGB channels will be copied from the target. This can be used, for example, to
	 * apply an alpha mask to a display object. This can also be used to combine a JPG compressed RGB image with a PNG32
	 * alpha mask, which can result in a much smaller file size than a single PNG32 containing ARGB.
	 *
	 * <b>IMPORTANT NOTE: This filter currently does not support the targetCtx, or targetX/Y parameters correctly.</b>
	 *
	 * <h4>Example</h4>
	 * This example draws a gradient box, then caches it and uses the "cacheCanvas" as the alpha mask on a 100x100 image.
	 *
	 *      var box = new createjs.Shape();
	 *      box.graphics.beginLinearGradientFill(["#000000", "rgba(0, 0, 0, 0)"], [0, 1], 0, 0, 100, 100)
	 *      box.graphics.drawRect(0, 0, 100, 100);
	 *      box.cache(0, 0, 100, 100);
	 *
	 *      var bmp = new createjs.Bitmap("path/to/image.jpg");
	 *      bmp.filters = [
	 *          new createjs.AlphaMaskFilter(box.cacheCanvas)
	 *      ];
	 *      bmp.cache(0, 0, 100, 100);
	 *
	 * See {{#crossLink "Filter"}}{{/crossLink}} for more information on applying filters.
	 * @class AlphaMaskFilter
	 * @extends Filter
	 * @constructor
	 * @param {Image} mask
	 **/
	var AlphaMaskFilter = function (mask) {
		this.initialize(mask);
	};
	var p = AlphaMaskFilter.prototype = new createjs.Filter();

// constructor:
	/** @ignore */
	p.initialize = function (mask) {
		this.mask = mask;
	};

// public properties:

	/**
	 * The image (or canvas) to use as the mask.
	 * @property mask
	 * @type Image
	 **/
	p.mask = null;

// public methods:

	/**
	 * Applies the filter to the specified context.
	 *
	 * <strong>IMPORTANT NOTE: This filter currently does not support the targetCtx, or targetX/Y parameters
	 * correctly.</strong>
	 * @method applyFilter
	 * @param {CanvasRenderingContext2D} ctx The 2D context to use as the source.
	 * @param {Number} x The x position to use for the source rect.
	 * @param {Number} y The y position to use for the source rect.
	 * @param {Number} width The width to use for the source rect.
	 * @param {Number} height The height to use for the source rect.
	 * @param {CanvasRenderingContext2D} [targetCtx] The 2D context to draw the result to. Defaults to the context passed to ctx.
	 * @param {Number} [targetX] The x position to draw the result to. Defaults to the value passed to x.
	 * @param {Number} [targetY] The y position to draw the result to. Defaults to the value passed to y.
	 * @return {Boolean} If the filter was applied successfully.
	 **/
	p.applyFilter = function (ctx, x, y, width, height, targetCtx, targetX, targetY) {
		if (!this.mask) {
			return true;
		}
		targetCtx = targetCtx || ctx;
		if (targetX == null) {
			targetX = x;
		}
		if (targetY == null) {
			targetY = y;
		}

		targetCtx.save();
		if (ctx != targetCtx) {
			// TODO: support targetCtx and targetX/Y
			// clearRect, then draw the ctx in?
		}

		targetCtx.globalCompositeOperation = "destination-in";
		targetCtx.drawImage(this.mask, targetX, targetY);
		targetCtx.restore();
		return true;
	};

	/**
	 * Returns a clone of this object.
	 * @method clone
	 * @return {AlphaMaskFilter}
	 **/
	p.clone = function () {
		return new AlphaMaskFilter(this.mask);
	};

	p.toString = function () {
		return "[AlphaMaskFilter]";
	};

// private methods:


	createjs.AlphaMaskFilter = AlphaMaskFilter;
}());
/*
* ColorFilter
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Applies a color transform to DisplayObjects.
 *
 * <h4>Example</h4>
 * This example draws a red circle, and then transforms it to Blue. This is accomplished by multiplying all the channels
 * to 0 (except alpha, which is set to 1), and then adding 255 to the blue channel.
 *
 *      var shape = new createjs.Shape().set({x:100,y:100});
 *      shape.graphics.beginFill("#ff0000").drawCircle(0,0,50);
 *
 *      shape.filters = [
 *          new createjs.ColorFilter(0,0,0,1, 0,0,255,0)
 *      ];
 *      shape.cache(-50, -50, 100, 100);
 *
 * See {{#crossLink "Filter"}}{{/crossLink}} for an more information on applying filters.
 * @class ColorFilter
 * @param {Number} [redMultiplier=1] The amount to multiply against the red channel. This is a range between 0 and 1.
 * @param {Number} [greenMultiplier=1] The amount to multiply against the green channel. This is a range between 0 and 1.
 * @param {Number} [blueMultiplier=1] The amount to multiply against the blue channel. This is a range between 0 and 1.
 * @param {Number} [alphaMultiplier=1] The amount to multiply against the alpha channel. This is a range between 0 and 1.
 * @param {Number} [redOffset=0] The amount to add to the red channel after it has been multiplied. This is a range
 * between -255 and 255.
 * @param {Number} [greenOffset=0] The amount to add to the green channel after it has been multiplied. This is a range
  * between -255 and 255.
 * @param {Number} [blueOffset=0] The amount to add to the blue channel after it has been multiplied. This is a range
  * between -255 and 255.
 * @param {Number} [alphaOffset=0] The amount to add to the alpha channel after it has been multiplied. This is a range
  * between -255 and 255.
 * @constructor
 * @extends Filter
 **/
var ColorFilter = function(redMultiplier, greenMultiplier, blueMultiplier, alphaMultiplier, redOffset, greenOffset, blueOffset, alphaOffset) {
  this.initialize(redMultiplier, greenMultiplier, blueMultiplier, alphaMultiplier, redOffset, greenOffset, blueOffset, alphaOffset);
}
var p = ColorFilter.prototype = new createjs.Filter();

// public properties:
	/**
	 * Red channel multiplier.
	 * @property redMultiplier
	 * @type Number
	 **/
	p.redMultiplier = 1;

	/**
	 * Green channel multiplier.
	 * @property greenMultiplier
	 * @type Number
	 **/
	p.greenMultiplier = 1;

	/**
	 * Blue channel multiplier.
	 * @property blueMultiplier
	 * @type Number
	 **/
	p.blueMultiplier = 1;

	/**
	 * Alpha channel multiplier.
	 * @property alphaMultiplier
	 * @type Number
	 **/
	p.alphaMultiplier = 1;

	/**
	 * Red channel offset (added to value).
	 * @property redOffset
	 * @type Number
	 **/
	p.redOffset = 0;

	/**
	 * Green channel offset (added to value).
	 * @property greenOffset
	 * @type Number
	 **/
	p.greenOffset = 0;

	/**
	 * Blue channel offset (added to value).
	 * @property blueOffset
	 * @type Number
	 **/
	p.blueOffset = 0;

	/**
	 * Alpha channel offset (added to value).
	 * @property alphaOffset
	 * @type Number
	 **/
	p.alphaOffset = 0;

// constructor:
	/**
	 * Initialization method.
	 * @method initialize
	 * @param {Number} [redMultiplier=1] The amount to multiply against the red channel. This is a range between 0 and 1.
	 * @param {Number} [greenMultiplier=1] The amount to multiply against the green channel. This is a range between 0 and 1.
	 * @param {Number} [blueMultiplier=1] The amount to multiply against the blue channel. This is a range between 0 and 1.
	 * @param {Number} [alphaMultiplier=1] The amount to multiply against the alpha channel. This is a range between 0 and 1.
	 * @param {Number} [redOffset=0] The amount to add to the red channel after it has been multiplied. This is a range
	 * between -255 and 255.
	 * @param {Number} [greenOffset=0] The amount to add to the green channel after it has been multiplied. This is a range
	 * between -255 and 255.
	 * @param {Number} [blueOffset=0] The amount to add to the blue channel after it has been multiplied. This is a range
	 * between -255 and 255.
	 * @param {Number} [alphaOffset=0] The amount to add to the alpha channel after it has been multiplied. This is a range
	 * between -255 and 255.
	 * @protected
	 **/
	p.initialize = function(redMultiplier, greenMultiplier, blueMultiplier, alphaMultiplier, redOffset, greenOffset, blueOffset, alphaOffset) {
		this.redMultiplier = redMultiplier != null ? redMultiplier : 1;
		this.greenMultiplier = greenMultiplier != null ? greenMultiplier : 1;
		this.blueMultiplier = blueMultiplier != null ? blueMultiplier : 1;
		this.alphaMultiplier = alphaMultiplier != null ? alphaMultiplier : 1;
		this.redOffset = redOffset || 0;
		this.greenOffset = greenOffset || 0;
		this.blueOffset = blueOffset || 0;
		this.alphaOffset = alphaOffset || 0;
	}

// public methods:
	p.applyFilter = function(ctx, x, y, width, height, targetCtx, targetX, targetY) {
		targetCtx = targetCtx || ctx;
		if (targetX == null) { targetX = x; }
		if (targetY == null) { targetY = y; }
		try {
			var imageData = ctx.getImageData(x, y, width, height);
		} catch(e) {
			//if (!this.suppressCrossDomainErrors) throw new Error("unable to access local image data: " + e);
			return false;
		}
		var data = imageData.data;
		var l = data.length;
		for (var i=0; i<l; i+=4) {
			data[i] = data[i]*this.redMultiplier+this.redOffset;
			data[i+1] = data[i+1]*this.greenMultiplier+this.greenOffset;
			data[i+2] = data[i+2]*this.blueMultiplier+this.blueOffset;
			data[i+3] = data[i+3]*this.alphaMultiplier+this.alphaOffset;
		}
		targetCtx.putImageData(imageData, targetX, targetY);
		return true;
	}

	p.toString = function() {
		return "[ColorFilter]";
	}

	/**
	 * Returns a clone of this ColorFilter instance.
	 * @method clone
	 * @return {ColorFilter} A clone of the current ColorFilter instance.
	 **/
	p.clone = function() {
		return new ColorFilter(this.redMultiplier, this.greenMultiplier, this.blueMultiplier, this.alphaMultiplier, this.redOffset, this.greenOffset, this.blueOffset, this.alphaOffset);
	}

	createjs.ColorFilter = ColorFilter;

}());
/*
* ColorMatrix
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

	/**
	 * Provides helper functions for assembling a matrix for use with the {{#crossLink "ColorMatrixFilter"}}{{/crossLink}},
	 * or can be used directly as the matrix for a ColorMatrixFilter. Most methods return the instance to facilitate
	 * chained calls.
	 *
	 * <h4>Example</h4>
	 *      myColorMatrix.adjustHue(20).adjustBrightness(50);
	 *
	 * See {{#crossLink "Filter"}}{{/crossLink}} for an example of how to apply filters, or {{#crossLink "ColorMatrixFilter"}}{{/crossLink}}
	 * for an example of how to use ColorMatrix to change a DisplayObject's color.
	 * @class ColorMatrix
	 * @param {Number} brightness
	 * @param {Number} contrast
	 * @param {Number} saturation
	 * @param {Number} hue
	 * @constructor
	 **/
	var ColorMatrix = function(brightness, contrast, saturation, hue) {
	  this.initialize(brightness, contrast, saturation, hue);
	};
	var p = ColorMatrix.prototype;

	/**
	 * Array of delta values for contrast calculations.
	 * @property DELTA_INDEX
	 * @type Array
	 * @protected
	 * @static
	 **/
	ColorMatrix.DELTA_INDEX = [
		0,    0.01, 0.02, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1,  0.11,
		0.12, 0.14, 0.15, 0.16, 0.17, 0.18, 0.20, 0.21, 0.22, 0.24,
		0.25, 0.27, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42,
		0.44, 0.46, 0.48, 0.5,  0.53, 0.56, 0.59, 0.62, 0.65, 0.68,
		0.71, 0.74, 0.77, 0.80, 0.83, 0.86, 0.89, 0.92, 0.95, 0.98,
		1.0,  1.06, 1.12, 1.18, 1.24, 1.30, 1.36, 1.42, 1.48, 1.54,
		1.60, 1.66, 1.72, 1.78, 1.84, 1.90, 1.96, 2.0,  2.12, 2.25,
		2.37, 2.50, 2.62, 2.75, 2.87, 3.0,  3.2,  3.4,  3.6,  3.8,
		4.0,  4.3,  4.7,  4.9,  5.0,  5.5,  6.0,  6.5,  6.8,  7.0,
		7.3,  7.5,  7.8,  8.0,  8.4,  8.7,  9.0,  9.4,  9.6,  9.8,
		10.0
	];

	/**
	 * Identity matrix values.
	 * @property IDENTITY_MATRIX
	 * @type Array
	 * @protected
	 * @static
	 **/
	ColorMatrix.IDENTITY_MATRIX = [
		1,0,0,0,0,
		0,1,0,0,0,
		0,0,1,0,0,
		0,0,0,1,0,
		0,0,0,0,1
	];

	/**
	 * The constant length of a color matrix.
	 * @property LENGTH
	 * @type Number
	 * @protected
	 * @static
	 **/
	ColorMatrix.LENGTH = ColorMatrix.IDENTITY_MATRIX.length;


	/**
	 * Initialization method.
	 * @method initialize
	 * @param {Number} brightness
	 * @param {Number} contrast
	 * @param {Number} saturation
	 * @param {Number} hue
	 * @protected
	 */
	p.initialize = function(brightness,contrast,saturation,hue) {
		this.reset();
		this.adjustColor(brightness,contrast,saturation,hue);
		return this;
	};

	/**
	 * Resets the matrix to identity values.
	 * @method reset
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 */
	p.reset = function() {
		return this.copyMatrix(ColorMatrix.IDENTITY_MATRIX);
	};

	/**
	 * Shortcut method to adjust brightness, contrast, saturation and hue.
	 * Equivalent to calling adjustHue(hue), adjustContrast(contrast),
	 * adjustBrightness(brightness), adjustSaturation(saturation), in that order.
	 * @method adjustColor
	 * @param {Number} brightness
	 * @param {Number} contrast
	 * @param {Number} saturation
	 * @param {Number} hue
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.adjustColor = function(brightness,contrast,saturation,hue) {
		this.adjustHue(hue);
		this.adjustContrast(contrast);
		this.adjustBrightness(brightness);
		return this.adjustSaturation(saturation);
	};

	/**
	 * Adjusts the brightness of pixel color by adding the specified value to the red, green and blue channels.
	 * Positive values will make the image brighter, negative values will make it darker.
	 * @method adjustBrightness
	 * @param {Number} value A value between -255 & 255 that will be added to the RGB channels.
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.adjustBrightness = function(value) {
		if (value == 0 || isNaN(value)) { return this; }
		value = this._cleanValue(value,255);
		this._multiplyMatrix([
			1,0,0,0,value,
			0,1,0,0,value,
			0,0,1,0,value,
			0,0,0,1,0,
			0,0,0,0,1
		]);
		return this;
	};

	/**
	 * Adjusts the contrast of pixel color.
	 * Positive values will increase contrast, negative values will decrease contrast.
	 * @method adjustContrast
	 * @param {Number} value A value between -100 & 100.
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.adjustContrast = function(value) {
		if (value == 0 || isNaN(value)) { return this; }
		value = this._cleanValue(value,100);
		var x;
		if (value<0) {
			x = 127+value/100*127;
		} else {
			x = value%1;
			if (x == 0) {
				x = ColorMatrix.DELTA_INDEX[value];
			} else {
				x = ColorMatrix.DELTA_INDEX[(value<<0)]*(1-x)+ColorMatrix.DELTA_INDEX[(value<<0)+1]*x; // use linear interpolation for more granularity.
			}
			x = x*127+127;
		}
		this._multiplyMatrix([
			x/127,0,0,0,0.5*(127-x),
			0,x/127,0,0,0.5*(127-x),
			0,0,x/127,0,0.5*(127-x),
			0,0,0,1,0,
			0,0,0,0,1
		]);
		return this;
	};

	/**
	 * Adjusts the color saturation of the pixel.
	 * Positive values will increase saturation, negative values will decrease saturation (trend towards greyscale).
	 * @method adjustSaturation
	 * @param {Number} value A value between -100 & 100.
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.adjustSaturation = function(value) {
		if (value == 0 || isNaN(value)) { return this; }
		value = this._cleanValue(value,100);
		var x = 1+((value > 0) ? 3*value/100 : value/100);
		var lumR = 0.3086;
		var lumG = 0.6094;
		var lumB = 0.0820;
		this._multiplyMatrix([
			lumR*(1-x)+x,lumG*(1-x),lumB*(1-x),0,0,
			lumR*(1-x),lumG*(1-x)+x,lumB*(1-x),0,0,
			lumR*(1-x),lumG*(1-x),lumB*(1-x)+x,0,0,
			0,0,0,1,0,
			0,0,0,0,1
		]);
		return this;
	};


	/**
	 * Adjusts the hue of the pixel color.
	 * @method adjustHue
	 * @param {Number} value A value between -180 & 180.
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.adjustHue = function(value) {
		if (value == 0 || isNaN(value)) { return this; }
		value = this._cleanValue(value,180)/180*Math.PI;
		var cosVal = Math.cos(value);
		var sinVal = Math.sin(value);
		var lumR = 0.213;
		var lumG = 0.715;
		var lumB = 0.072;
		this._multiplyMatrix([
			lumR+cosVal*(1-lumR)+sinVal*(-lumR),lumG+cosVal*(-lumG)+sinVal*(-lumG),lumB+cosVal*(-lumB)+sinVal*(1-lumB),0,0,
			lumR+cosVal*(-lumR)+sinVal*(0.143),lumG+cosVal*(1-lumG)+sinVal*(0.140),lumB+cosVal*(-lumB)+sinVal*(-0.283),0,0,
			lumR+cosVal*(-lumR)+sinVal*(-(1-lumR)),lumG+cosVal*(-lumG)+sinVal*(lumG),lumB+cosVal*(1-lumB)+sinVal*(lumB),0,0,
			0,0,0,1,0,
			0,0,0,0,1
		]);
		return this;
	};

	/**
	 * Concatenates (multiplies) the specified matrix with this one.
	 * @method concat
	 * @param {Array} matrix An array or ColorMatrix instance.
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.concat = function(matrix) {
		matrix = this._fixMatrix(matrix);
		if (matrix.length != ColorMatrix.LENGTH) { return this; }
		this._multiplyMatrix(matrix);
		return this;
	};

	/**
	 * Returns a clone of this ColorMatrix.
	 * @method clone
	 * @return {ColorMatrix} A clone of this ColorMatrix.
	 **/
	p.clone = function() {
		return (new ColorMatrix()).copyMatrix(this);
	};

	/**
	 * Return a length 25 (5x5) array instance containing this matrix's values.
	 * @method toArray
	 * @return {Array} An array holding this matrix's values.
	 **/
	p.toArray = function() {
		var arr = [];
		for (var i= 0, l=ColorMatrix.LENGTH; i<l; i++) {
			arr[i] = this[i];
		}
		return arr;
	};

	/**
	 * Copy the specified matrix's values to this matrix.
	 * @method copyMatrix
	 * @param {Array} matrix An array or ColorMatrix instance.
	 * @return {ColorMatrix} The ColorMatrix instance the method is called on (useful for chaining calls.)
	 **/
	p.copyMatrix = function(matrix) {
		var l = ColorMatrix.LENGTH;
		for (var i=0;i<l;i++) {
			this[i] = matrix[i];
		}
		return this;
	};
	
	/**
	 * Returns a string representation of this object.
	 * @method toString
	 * @return {String} a string representation of the instance.
	 **/
	p.toString = function() {
		return "[ColorMatrix]";
	};

// private methods:

	/**
	 * @method _multiplyMatrix
	 * @param {Array} matrix
	 * @protected
	 **/
	p._multiplyMatrix = function(matrix) {
		var col = [];

		for (var i=0;i<5;i++) {
			for (var j=0;j<5;j++) {
				col[j] = this[j+i*5];
			}
			for (var j=0;j<5;j++) {
				var val=0;
				for (var k=0;k<5;k++) {
					val += matrix[j+k*5]*col[k];
				}
				this[j+i*5] = val;
			}
		}
	};

	/**
	 * Make sure values are within the specified range, hue has a limit of 180, brightness is 255, others are 100.
	 * @method _cleanValue
	 * @param {Number} value The raw number
	 * @param {Number} limit The maximum that the number can be. The minimum is the limit * -1.
	 * @protected
	 **/
	p._cleanValue = function(value, limit) {
		return Math.min(limit,Math.max(-limit,value));
	};

	//
	/**
	 * Makes sure matrixes are 5x5 (25 long).
	 * @method _fixMatrix
	 * @param {Array} matrix
	 * @protected
	 **/
	p._fixMatrix = function(matrix) {
		if (matrix instanceof ColorMatrix) { matrix = matrix.toArray(); }
		if (matrix.length < ColorMatrix.LENGTH) {
			matrix = matrix.slice(0,matrix.length).concat(ColorMatrix.IDENTITY_MATRIX.slice(matrix.length,ColorMatrix.LENGTH));
		} else if (matrix.length > ColorMatrix.LENGTH) {
			matrix = matrix.slice(0,ColorMatrix.LENGTH);
		}
		return matrix;
	};

	createjs.ColorMatrix = ColorMatrix;

}());
/*
* ColorMatrixFilter
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

/**
 * Allows you to carry out complex color operations such as modifying saturation, brightness, or inverting. See the
 * {{#crossLink "ColorMatrix"}}{{/crossLink}} for more information on changing colors. For an easier color transform,
 * consider the {{#crossLink "ColorFilter"}}{{/crossLink}}.
 *
 * <h4>Example</h4>
 * This example creates a red circle, inverts its hue, and then saturates it to brighten it up.
 *
 *      var shape = new createjs.Shape().set({x:100,y:100});
 *      shape.graphics.beginFill("#ff0000").drawCircle(0,0,50);
 *
 *      var matrix = new createjs.ColorMatrix().adjustHue(180).adjustSaturation(100);
 *      shape.filters = [
 *          new createjs.ColorMatrixFilter(matrix)
 *      ];
 *
 *      shape.cache(-50, -50, 100, 100);
 *
 * See {{#crossLink "Filter"}}{{/crossLink}} for an more information on applying filters.
 * @class ColorMatrixFilter
 * @constructor
 * @extends Filter
 * @param {Array} matrix A 4x5 matrix describing the color operation to perform. See also the {{#crossLink "ColorMatrix"}}{{/crossLink}}
 * class.
 **/
var ColorMatrixFilter = function(matrix) {
  this.initialize(matrix);
};
var p = ColorMatrixFilter.prototype = new createjs.Filter();

// public properties:
	p.matrix = null;

// constructor:
	// TODO: detailed docs.
	/**
	 * @method initialize
	 * @protected
	 * @param {Array} matrix A 4x5 matrix describing the color operation to perform.
	 **/
	p.initialize = function(matrix) {
		this.matrix = matrix;
	};

// public methods:
	p.applyFilter = function(ctx, x, y, width, height, targetCtx, targetX, targetY) {
		targetCtx = targetCtx || ctx;
		if (targetX == null) { targetX = x; }
		if (targetY == null) { targetY = y; }
		try {
			var imageData = ctx.getImageData(x, y, width, height);
		} catch(e) {
			//if (!this.suppressCrossDomainErrors) throw new Error("unable to access local image data: " + e);
			return false;
		}
		var data = imageData.data;
		var l = data.length;
		var r,g,b,a;
		var mtx = this.matrix;
		var m0 =  mtx[0],  m1 =  mtx[1],  m2 =  mtx[2],  m3 =  mtx[3],  m4 =  mtx[4];
		var m5 =  mtx[5],  m6 =  mtx[6],  m7 =  mtx[7],  m8 =  mtx[8],  m9 =  mtx[9];
		var m10 = mtx[10], m11 = mtx[11], m12 = mtx[12], m13 = mtx[13], m14 = mtx[14];
		var m15 = mtx[15], m16 = mtx[16], m17 = mtx[17], m18 = mtx[18], m19 = mtx[19];

		for (var i=0; i<l; i+=4) {
			r = data[i];
			g = data[i+1];
			b = data[i+2];
			a = data[i+3];
			data[i] = r*m0+g*m1+b*m2+a*m3+m4; // red
			data[i+1] = r*m5+g*m6+b*m7+a*m8+m9; // green
			data[i+2] = r*m10+g*m11+b*m12+a*m13+m14; // blue
			data[i+3] = r*m15+g*m16+b*m17+a*m18+m19; // alpha
		}
		targetCtx.putImageData(imageData, targetX, targetY);
		return true;
	};

	p.toString = function() {
		return "[ColorMatrixFilter]";
	};

	/**
	 * Returns a clone of this ColorMatrixFilter instance.
	 * @method clone
	 * @return {ColorMatrixFilter} A clone of the current ColorMatrixFilter instance.
	 **/
	p.clone = function() {
		return new ColorMatrixFilter(this.matrix);
	};

	createjs.ColorMatrixFilter = ColorMatrixFilter;

}());
/*
* Touch
* Visit http://createjs.com/ for documentation, updates and examples.
*
* Copyright (c) 2010 gskinner.com, inc.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following
* conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @module EaselJS
 */

// namespace:
this.createjs = this.createjs||{};

(function() {
	"use strict";

// TODO: support for double tap.
/**
 * Global utility for working with multi-touch enabled devices in EaselJS. Currently supports W3C Touch API (iOS and
 * modern Android browser) and the Pointer API (IE), including ms-prefixed events in IE10, and unprefixed in IE11.
 *
 * Ensure that you {{#crossLink "Touch/disable"}}{{/crossLink}} touch when cleaning up your application. You do not have
 * to check if touch is supported to enable it, as it will fail gracefully if it is not supported.
 *
 * <h4>Example</h4>
 *
 *      var stage = new createjs.Stage("canvasId");
 *      createjs.Touch.enable(stage);
 *
 * <strong>Note:</strong> It is important to disable Touch on a stage that you are no longer using:
 *
 *      createjs.Touch.disable(stage);
 *
 * @class Touch
 * @static
 **/
var Touch = function() {
	throw "Touch cannot be instantiated";
};

// Public static methods:
	/**
	 * Returns `true` if touch is supported in the current browser.
	 * @method isSupported
	 * @return {Boolean} Indicates whether touch is supported in the current browser.
	 * @static
	 **/
	Touch.isSupported = function() {
		return	('ontouchstart' in window) // iOS
			|| (window.navigator['msPointerEnabled'] && window.navigator['msMaxTouchPoints'] > 0) // IE10
			|| (window.navigator['pointerEnabled'] && window.navigator['maxTouchPoints'] > 0); // IE11+
	};

	/**
	 * Enables touch interaction for the specified EaselJS {{#crossLink "Stage"}}{{/crossLink}}. Currently supports iOS
	 * (and compatible browsers, such as modern Android browsers), and IE10/11. Supports both single touch and
	 * multi-touch modes. Extends the EaselJS {{#crossLink "MouseEvent"}}{{/crossLink}} model, but without support for
	 * double click or over/out events. See the MouseEvent {{#crossLink "MouseEvent/pointerId:property"}}{{/crossLink}}
	 * for more information.
	 * @method enable
	 * @param {Stage} stage The {{#crossLink "Stage"}}{{/crossLink}} to enable touch on.
	 * @param {Boolean} [singleTouch=false] If `true`, only a single touch will be active at a time.
	 * @param {Boolean} [allowDefault=false] If `true`, then default gesture actions (ex. scrolling, zooming) will be
	 * allowed when the user is interacting with the target canvas.
	 * @return {Boolean} Returns `true` if touch was successfully enabled on the target stage.
	 * @static
	 **/
	Touch.enable = function(stage, singleTouch, allowDefault) {
		if (!stage || !stage.canvas || !Touch.isSupported()) { return false; }

		// inject required properties on stage:
		stage.__touch = {pointers:{}, multitouch:!singleTouch, preventDefault:!allowDefault, count:0};

		// note that in the future we may need to disable the standard mouse event model before adding
		// these to prevent duplicate calls. It doesn't seem to be an issue with iOS devices though.
		if ('ontouchstart' in window) { Touch._IOS_enable(stage); }
		else if (window.navigator['msPointerEnabled'] || window.navigator["pointerEnabled"]) { Touch._IE_enable(stage); }
		return true;
	};

	/**
	 * Removes all listeners that were set up when calling `Touch.enable()` on a stage.
	 * @method disable
	 * @param {Stage} stage The {{#crossLink "Stage"}}{{/crossLink}} to disable touch on.
	 * @static
	 **/
	Touch.disable = function(stage) {
		if (!stage) { return; }
		if ('ontouchstart' in window) { Touch._IOS_disable(stage); }
		else if (window.navigator['msPointerEnabled'] || window.navigator["pointerEnabled"]) { Touch._IE_disable(stage); }
	};

// Private static methods:

	/**
	 * @method _IOS_enable
	 * @protected
	 * @param {Stage} stage
	 * @static
	 **/
	Touch._IOS_enable = function(stage) {
		var canvas = stage.canvas;
		var f = stage.__touch.f = function(e) { Touch._IOS_handleEvent(stage,e); };
		canvas.addEventListener("touchstart", f, false);
		canvas.addEventListener("touchmove", f, false);
		canvas.addEventListener("touchend", f, false);
		canvas.addEventListener("touchcancel", f, false);
	};

	/**
	 * @method _IOS_disable
	 * @protected
	 * @param {Stage} stage
	 * @static
	 **/
	Touch._IOS_disable = function(stage) {
		var canvas = stage.canvas;
		if (!canvas) { return; }
		var f = stage.__touch.f;
		canvas.removeEventListener("touchstart", f, false);
		canvas.removeEventListener("touchmove", f, false);
		canvas.removeEventListener("touchend", f, false);
		canvas.removeEventListener("touchcancel", f, false);
	};

	/**
	 * @method _IOS_handleEvent
	 * @param {Stage} stage
	 * @param {Object} e The event to handle
	 * @protected
	 * @static
	 **/
	Touch._IOS_handleEvent = function(stage, e) {
		if (!stage) { return; }
		if (stage.__touch.preventDefault) { e.preventDefault&&e.preventDefault(); }
		var touches = e.changedTouches;
		var type = e.type;
		for (var i= 0,l=touches.length; i<l; i++) {
			var touch = touches[i];
			var id = touch.identifier;
			if (touch.target != stage.canvas) { continue; }

			if (type == "touchstart") {
				this._handleStart(stage, id, e, touch.pageX, touch.pageY);
			} else if (type == "touchmove") {
				this._handleMove(stage, id, e, touch.pageX, touch.pageY);
			} else if (type == "touchend" || type == "touchcancel") {
				this._handleEnd(stage, id, e);
			}
		}
	};

	/**
	 * @method _IE_enable
	 * @protected
	 * @param {Stage} stage
	 * @static
	 **/
	Touch._IE_enable = function(stage) {
		var canvas = stage.canvas;
		var f = stage.__touch.f = function(e) { Touch._IE_handleEvent(stage,e); };

		if (window.navigator["pointerEnabled"] === undefined) {
			canvas.addEventListener("MSPointerDown", f, false);
			window.addEventListener("MSPointerMove", f, false);
			window.addEventListener("MSPointerUp", f, false);
			window.addEventListener("MSPointerCancel", f, false);
			if (stage.__touch.preventDefault) { canvas.style.msTouchAction = "none"; }
		} else {
			canvas.addEventListener("pointerdown", f, false);
			window.addEventListener("pointermove", f, false);
			window.addEventListener("pointerup", f, false);
			window.addEventListener("pointercancel", f, false);
			if (stage.__touch.preventDefault) { canvas.style.touchAction = "none"; }

		}
		stage.__touch.activeIDs = {};
	};

	/**
	 * @method _IE_disable
	 * @protected
	 * @param {Stage} stage
	 * @static
	 **/
	Touch._IE_disable = function(stage) {
		var f = stage.__touch.f;

		if (window.navigator["pointerEnabled"] === undefined) {
			window.removeEventListener("MSPointerMove", f, false);
			window.removeEventListener("MSPointerUp", f, false);
			window.removeEventListener("MSPointerCancel", f, false);
			if (stage.canvas) {
				stage.canvas.removeEventListener("MSPointerDown", f, false);
			}
		} else {
			window.removeEventListener("pointermove", f, false);
			window.removeEventListener("pointerup", f, false);
			window.removeEventListener("pointercancel", f, false);
			if (stage.canvas) {
				stage.canvas.removeEventListener("pointerdown", f, false);
			}
		}
	};

	/**
	 * @method _IE_handleEvent
	 * @param {Stage} stage
	 * @param {Object} e The event to handle.
	 * @protected
	 * @static
	 **/
	Touch._IE_handleEvent = function(stage, e) {
		if (!stage) { return; }
		if (stage.__touch.preventDefault) { e.preventDefault && e.preventDefault(); }
		var type = e.type;
		var id = e.pointerId;
		var ids = stage.__touch.activeIDs;

		if (type == "MSPointerDown" || type == "pointerdown") {
			if (e.srcElement != stage.canvas) { return; }
			ids[id] = true;
			this._handleStart(stage, id, e, e.pageX, e.pageY);
		} else if (ids[id]) { // it's an id we're watching
			if (type == "MSPointerMove" || type == "pointermove") {
				this._handleMove(stage, id, e, e.pageX, e.pageY);
			} else if (type == "MSPointerUp" || type == "MSPointerCancel"
					|| type == "pointerup" || type == "pointercancel") {
				delete(ids[id]);
				this._handleEnd(stage, id, e);
			}
		}
	};


	/**
	 * @method _handleStart
	 * @param {Stage} stage
	 * @param {String|Number} id
	 * @param {Object} e
	 * @param {Number} x
	 * @param {Number} y
	 * @protected
	 **/
	Touch._handleStart = function(stage, id, e, x, y) {
		var props = stage.__touch;
		if (!props.multitouch && props.count) { return; }
		var ids = props.pointers;
		if (ids[id]) { return; }
		ids[id] = true;
		props.count++;
		stage._handlePointerDown(id, e, x, y);
	};

	/**
	 * @method _handleMove
	 * @param {Stage} stage
	 * @param {String|Number} id
	 * @param {Object} e
	 * @param {Number} x
	 * @param {Number} y
	 * @protected
	 **/
	Touch._handleMove = function(stage, id, e, x, y) {
		if (!stage.__touch.pointers[id]) { return; }
		stage._handlePointerMove(id, e, x, y);
	};

	/**
	 * @method _handleEnd
	 * @param {Stage} stage
	 * @param {String|Number} id
	 * @param {Object} e
	 * @protected
	 **/
	Touch._handleEnd = function(stage, id, e) {
		// TODO: cancel should be handled differently for proper UI (ex. an up would trigger a click, a cancel would more closely resemble an out).
		var props = stage.__touch;
		var ids = props.pointers;
		if (!ids[id]) { return; }
		props.count--;
		stage._handlePointerUp(id, e, true);
		delete(ids[id]);
	};


createjs.Touch = Touch;
}());
/**
 * @module EaselJS
 */
this.createjs = this.createjs || {};

(function() {
	"use strict";

	/**
	 * Static class holding library specific information such as the version and buildDate of
	 * the library.
	 * @class EaselJS
	 **/
	var s = createjs.EaselJS = createjs.EaselJS || {};

	/**
	 * The version string for this release.
	 * @property version
	 * @type String
	 * @static
	 **/
	s.version = /*version*/"0.7.1"; // injected by build process

	/**
	 * The build date for this release in UTC format.
	 * @property buildDate
	 * @type String
	 * @static
	 **/
	s.buildDate = /*date*/"Mon, 28 Jul 2014 17:23:09 GMT"; // injected by build process

})();
