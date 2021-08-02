/**
 * @format
 */

import {CancelledError} from '../util/Error.js';

export class FileLoader {
  constructor(options) {
    this._dropper = this._createDropper(options.classNames);
    this._picker = this._createPicker(options.classNames);
    this._preventDefault = event => event.preventDefault();
    this._resolve = null;
    this._reject = null;
  }

  get isActive() {
    return window.document.body.contains(this._dropper);
  }

  loadFile() {
    this._setTextContent('file');
    return this._getFiles().then(files => files[0]);
  }

  async loadFiles() {
    this._setTextContent('files');
    const input = this._picker.lastElementChild;
    input.setAttribute('multiple', 'multiple');

    const files = await this._getFiles();
    input.removeAttribute('multiple');

    const promises = [];
    for (const file of files) {
      promises.push(
        this._parseFile(file).then(data => ({
          children: [],
          data,
          name: file.name,
          type: null,
          url: `file:${file.name}`,
        })),
      );
    }

    const parsed = await Promise.allSettled(promises);
    return parsed
      .filter(item => item.status === 'fulfilled')
      .map(item => Promise.resolve(item.value));
  }

  hide() {
    const preventDefault = this._preventDefault;
    const document = window.document;
    document.removeEventListener('dragover', preventDefault);
    document.removeEventListener('drop', preventDefault);

    const body = document.body;
    body.removeChild(this._dropper);
    body.removeChild(this._picker);

    this._resolve = null;
    const reject = this._reject;
    if (!!reject) {
      this._reject = null;
      reject(new CancelledError('Cancelled file load'));
    }
  }

  show() {
    const preventDefault = this._preventDefault;
    const document = window.document;
    document.addEventListener('dragover', preventDefault);
    document.addEventListener('drop', preventDefault);

    const body = document.body;
    const dropper = this._dropper;
    const picker = this._picker;
    body.appendChild(picker);
    body.appendChild(dropper);
  }

  _createDropper(classNames) {
    const document = window.document;
    const span = document.createElement('span');
    span.classList.add('opensfm-file-text');

    const dropper = document.createElement('div');
    dropper.classList.add('opensfm-file-drop', ...classNames);
    dropper.appendChild(span);

    dropper.addEventListener('dragenter', event => {
      dropper.classList.add('opensfm-file-drop-hover');
      event.preventDefault();
    });
    dropper.addEventListener('dragleave', event => {
      dropper.classList.remove('opensfm-file-drop-hover');
      event.preventDefault();
    });
    dropper.addEventListener('dragover', event => {
      event.dataTransfer.dropEffect = 'copy';
      event.preventDefault();
    });

    return dropper;
  }

  _createPicker(classNames) {
    const document = window.document;
    const span = document.createElement('span');
    span.classList.add('opensfm-file-text');

    const input = document.createElement('input');
    input.type = 'file';

    const picker = document.createElement('label');
    picker.classList.add('opensfm-file-pick', ...classNames);
    picker.appendChild(span);
    picker.appendChild(input);

    return picker;
  }

  _getFiles() {
    const promise = new Promise((resolve, reject) => {
      this._resolve = resolve;
      this._reject = reject;

      this._picker.addEventListener('change', this._onChange);
      this._dropper.addEventListener('drop', this._onDrop);
    });

    return promise;
  }

  _onChange = event => {
    const resolve = this._resolve;
    this._resolve = null;
    this._reject = null;

    this._picker.removeEventListener('change', this._onChange);
    resolve(event.target.files);
  };

  _onDrop = event => {
    const resolve = this._resolve;
    const reject = this._reject;
    this._resolve = null;
    this._reject = null;

    this._dropper.removeEventListener('drop', this._onDrop);
    this._dropper.classList.remove('opensfm-file-drop-hover');
    event.preventDefault();
    const items = event.dataTransfer.items;
    if (!items) {
      reject(new Error('No files loaded'));
    }

    const files = [];
    for (const item of items) {
      if (!this._verifyKind(item.kind)) {
        continue;
      }
      files.push(item.getAsFile());
    }

    if (files.length > 0) {
      resolve(files);
    }
    reject(new Error('No files loaded'));
  };

  _parseFile(file) {
    return new Promise((resolve, reject) => {
      if (file.type !== 'application/json') {
        reject(new Error(`Unrecognized file type: ${file.type}`));
      }

      const reader = new FileReader();
      reader.addEventListener('load', event => {
        try {
          const data = JSON.parse(event.target.result);
          const name = file.name;
          const type = file.type;
          const size = Math.round(file.size / 1024).toLocaleString();
          console.log(`File parsed: ${name} (${type}, ${size} kB)`);
          resolve(data);
        } catch (error) {
          reject(error);
        }
      });
      reader.addEventListener('error', event => reject(event));
      reader.readAsText(file);
    });
  }

  _setTextContent(content) {
    this._picker.firstElementChild.textContent = `Pick ${content}`;
    this._dropper.firstElementChild.textContent = `Drop ${content}`;
  }

  _verifyKind(kind) {
    if (kind !== 'file') {
      console.warn(`Unrecognized format ${kind}`);
      return false;
    }
    return true;
  }
}
