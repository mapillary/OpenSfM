/**
 * @format
 */

import {Popup} from './Popup.js';

export class CommandExplainerControl {
  constructor(options) {
    this._container = this._createContainer();
    this._popup = this._createPopup();
    this._visible = false;
    if (options.visible) {
      this.show();
    }
  }

  get container() {
    return this._container;
  }

  add(commands) {
    const lines = Object.keys(commands).map(key => {
      const value = commands[key].value;
      return `'${key}' - ${value}`;
    });
    this._popup.appendLines(lines);
  }

  hide() {
    if (!this._visible) {
      return;
    }
    this._container.classList.add('opensfm-hidden');
    this._visible = false;
  }

  show() {
    if (this._visible) {
      return;
    }
    this._container.classList.remove('opensfm-hidden');
    this._visible = true;
  }

  _createContainer() {
    const header = document.createElement('span');
    header.classList.add('opensfm-info-text', 'opensfm-info-text-header');
    header.textContent = 'Commands';

    const container = document.createElement('div');
    container.classList.add('opensfm-control-container', 'opensfm-hidden');
    container.appendChild(header);
    return container;
  }

  _createPopup() {
    return new Popup({
      container: this._container.firstElementChild,
      up: false,
    });
  }
}
