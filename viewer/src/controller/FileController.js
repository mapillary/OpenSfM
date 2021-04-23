/**
 * @format
 */

import {CancelledError} from '../util/Error.js';
import {EventEmitter} from '../util/EventEmitter.js';
import {FileLoader} from '../ui/FileLoader.js';
import {FileSelecter} from '../ui/FileSelecter.js';

export class FileController extends EventEmitter {
  constructor(options) {
    super();
    this._background = this._createBackground();

    if (options.showExitButton) {
      this._background.appendChild(this._createExitButton());
    }
    this._loader = new FileLoader({classNames: options.classNames});
    this._selecter = new FileSelecter(options);

    this._config = {loadFiles: async () => await this.toggle()};
  }

  addToFolder(folder) {
    this._controller = folder.add(this._config, 'loadFiles');
    this._controller.name('Load file');
  }

  isActive() {
    return this._loader.isActive && this._selecter.isActive;
  }

  async toggle() {
    const isActive = this.isActive();
    if (this._controller) {
      const name = isActive ? 'Load file' : 'Hide load';
      this._controller.name(name);
    }

    if (isActive) {
      this._hide();
      return;
    }

    this._show();
    await this._emit();
  }

  _createBackground() {
    const background = document.createElement('div');
    background.classList.add('opensfm-file-background');
    return background;
  }

  _createExitButton() {
    const button = document.createElement('div');
    button.classList.add('opensfm-file-exit-button');
    button.addEventListener('click', () => {
      if (this.isActive()) {
        this.toggle();
      }
    });
    return button;
  }

  async _emit() {
    const loader = this._loader;
    const selecter = this._selecter;
    let tries = 0;
    while (this.isActive()) {
      if (tries > 10) {
        console.warn('To many load attempts, aborting');
        this._hide();
        return;
      }
      try {
        const items = await Promise.race([
          selecter.selectFiles().then(files => ({files, target: selecter})),
          loader.loadFiles().then(files => ({files, target: loader})),
        ]);
        const target = items.target;
        const settled = await Promise.allSettled(
          items.files.map(p =>
            p.then(file => this.fire('load', {file, target})),
          ),
        );

        const succeeded = settled.reduce(
          (acc, curr) => acc || curr.status === 'fulfilled',
          false,
        );
        if (!succeeded) {
          throw new Error('Failed to load files');
        }
      } catch (error) {
        tries++;
        if (error instanceof CancelledError) {
          continue;
        }
        console.error(error);
        continue;
      }
    }
  }

  _hide() {
    document.body.removeChild(this._background);
    this._loader.hide();
    this._selecter.hide();
  }

  _show() {
    document.body.appendChild(this._background);
    this._loader.show();
    this._selecter.show();
  }
}
