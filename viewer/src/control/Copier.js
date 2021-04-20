/**
 * @format
 */

import {Popup} from './Popup.js';

export class Copier {
  constructor(options) {
    if (!window.navigator || !window.navigator.clipboard) {
      // Clipboard requires secure origin (https or localhost)
      // or setting a browser flag.
      return;
    }

    this._copyText = options.copyText;
    const container = options.container;
    this._resetCursor = container.style.cursor;
    container.style.cursor = 'pointer';
    container.addEventListener('click', this._onClick);

    this._popup = new Popup({container, up: true});
    this._popup.setLines(['Copy']);
    this._container = container;
  }

  dispose() {
    if (!this._container) {
      return;
    }
    this._popup.dispose();
    this._popup = null;
    const container = this._container;
    container.removeEventListener('click', this._onClick);
    container.style.cursor = this._resetCursor;
    this._container = null;
  }

  setCopyText(content) {
    this._copyText = content;
  }

  _onClick = async () => {
    try {
      const navigator = window.navigator;
      await navigator.clipboard.writeText(this._copyText);
      await this._showCopied();
    } catch (error) {
      console.error(error);
    }
  };

  _showCopied() {
    if (!this._popup) {
      return;
    }
    this._popup.setLines(['Copied to clipboard']);
    return new Promise(resolve => {
      window.setTimeout(() => {
        if (!this._popup) {
          resolve();
          return;
        }
        this._popup.setLines(['Copy']);
        resolve();
      }, 850);
    });
  }
}
