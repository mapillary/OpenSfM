/**
 * @format
 */

export class EventEmitter {
  constructor() {
    this._listeners = {};
  }

  on(type, callback) {
    if (!(type in this._listeners)) {
      this._listeners[type] = [];
    }
    this._listeners[type].push(callback);
  }

  fire(type, event) {
    if (!(type in this._listeners)) {
      return;
    }
    for (const callback of this._listeners[type]) {
      callback.call(this, event);
    }
  }
}
