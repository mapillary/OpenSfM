/**
 * @format
 */

export class Popup {
  constructor(options) {
    const container = options.container;

    const onShowTypes = ['pointerenter'];
    for (const type of onShowTypes) {
      container.addEventListener(type, this._onShow);
    }

    window.addEventListener('blur', this._onHide);
    const onHideTypes = ['pointercancel', 'pointerleave'];
    for (const type of onHideTypes) {
      container.addEventListener(type, this._onHide);
    }

    this._onHideTypes = onHideTypes;
    this._onShowTypes = onShowTypes;

    this._container = container;
    this._lines = [];
    this._up = options.up;
    this._popup = null;
    this._popupContainer = null;
  }

  appendLines(lines) {
    this._lines.push(...lines);
    this._appendLines(lines.slice());
  }

  setLines(lines) {
    this._lines = lines.slice();
    this._clearLines();
    this._appendLines(this._lines);
  }

  dispose() {
    if (!this._container) {
      return;
    }

    this._onHide();

    const container = this._container;
    container.style.cursor = this._resetCursor;

    const onShowTypes = this._onShowTypes;
    for (const type in onShowTypes) {
      container.removeEventListener(type, this._onShow);
    }

    window.removeEventListener('blur', this._onHide);
    const onHideTypes = this._onHideTypes;
    for (const type in onHideTypes) {
      container.removeEventListener(type, this._onHide);
    }

    this._onHideTypes = null;
    this._onShowTypes = null;
    this._container = null;
  }

  _appendLines(lines) {
    const popup = this._popup;
    if (!popup) {
      return;
    }
    for (const line of lines) {
      const span = document.createElement('span');
      span.style.display = 'block';
      span.textContent = line;
      popup.appendChild(span);
    }
  }

  _clearLines() {
    const popup = this._popup;
    if (!popup) {
      return;
    }
    while (popup.lastElementChild) {
      popup.removeChild(popup.lastElementChild);
    }
  }

  _onShow = () => {
    if (!!this._popup) {
      return;
    }
    const container = this._container;
    const left = container.offsetLeft;
    const boundingRect = container.getBoundingClientRect();
    const up = this._up;
    const top = up
      ? container.offsetTop
      : container.offsetTop + boundingRect.height;

    const directionClassname = up ? 'opensfm-popup-up' : 'opensfm-popup-down';
    const document = window.document;
    const popup = document.createElement('div');
    popup.classList.add('opensfm-popup');

    const arrow = document.createElement('div');
    arrow.classList.add('opensfm-popup-arrow', directionClassname);

    const popupContainer = document.createElement('div');
    popupContainer.classList.add('opensfm-popup-container', directionClassname);
    popupContainer.style.position = 'absolute';
    popupContainer.style.left = `${left + 2}px`;
    popupContainer.style.top = `${top}px`;
    popupContainer.appendChild(popup);
    if (up) {
      popupContainer.appendChild(arrow);
    } else {
      popup.before(arrow);
    }
    container.parentNode.appendChild(popupContainer);

    this._popupContainer = popupContainer;
    this._popup = popup;
    this._appendLines(this._lines);
  };

  _onHide = () => {
    const popupContainer = this._popupContainer;
    if (!popupContainer) {
      return;
    }
    this._container.parentNode.removeChild(popupContainer);
    this._popupContainer = null;
    this._popup = null;
  };
}
