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

    this._content = options.content;
    this._message = options.message ?? ['Copy'];

    const container = options.container;
    this._resetCursor = container.style.cursor;
    container.style.cursor = 'pointer';
    container.addEventListener('click', this._onClick);

    const up = options.up === false ? false : true;
    this._popup = new Popup({container, up});
    this._popup.setLines(this._message);
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

  setContent(content) {
    this._content = content;
  }

  setMessage(message) {
    if (!(message instanceof Array)) {
      throw new Error('Faulty message type');
    }
    this._message = message;
  }

  _onClick = async () => {
    try {
      const navigator = window.navigator;
      await navigator.clipboard.writeText(this._content);
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
        this._popup.setLines(this._message);
        resolve();
      }, 850);
    });
  }
}
