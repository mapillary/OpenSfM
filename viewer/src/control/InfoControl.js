/**
 * @format
 */

export class InfoControl {
  constructor(options) {
    const container = document.createElement('div');
    container.classList.add('opensfm-info-container');
    options.beforeContainer.after(container);
    this._container = container;
    this._controls = [];
  }

  addControl(control) {
    this._container.appendChild(control.container);
    this._controls.push(control);
  }

  setWidth(value) {
    const width = `${100 * value}%`;
    this._container.style.width = width;
  }
}
