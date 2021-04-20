/**
 * @format
 */

import {CancelledError} from '../util/Error.js';

function* traverse(roots) {
  for (const root of roots) {
    yield root;
    yield* traverse(root.children);
  }
}

function select(root, value) {
  root.selected = value;
  for (const child of root.children) {
    select(child, value);
  }
}

export class FileSelecter {
  constructor(options) {
    this._itemsUrl = options.itemsUrl;
    this._selecter = this._createSelecter(options.classNames);
    this._submit = this._createSubmit();
    this._items = new Map();
    this._resolver = null;
    this._rejecter = null;
  }

  get isActive() {
    return window.document.body.contains(this._selecter);
  }

  hide() {
    window.document.body.removeChild(this._selecter);

    this._resolver = null;
    const rejecter = this._rejecter;
    if (!!rejecter) {
      this._rejecter = null;
      rejecter(new CancelledError('Cancelled file selection'));
    }
  }

  selectFiles() {
    this._selecter.classList.remove('opensfm-file-select-inactive');
    const promise = new Promise(async (resolve, reject) => {
      this._resolver = resolve;
      this._rejecter = reject;
    });

    if (this._items.size === 0) {
      this._fetchList();
    }

    return promise;
  }

  show() {
    window.document.body.appendChild(this._selecter);
  }

  _clear() {
    this._resolver = null;
    this._rejecter = null;
    for (const item of this._items.values()) {
      select(item, false);
      item.container.classList.remove('opensfm-file-select-selected');
    }
  }

  _createItemContainer(item) {
    const document = window.document;
    const span = document.createElement('span');
    span.classList.add('opensfm-file-select-item');
    span.textContent = item.name;
    span.addEventListener('click', this._onItemClick);
    return span;
  }

  _createCategoryHeader(category) {
    const document = window.document;
    const span = document.createElement('span');
    span.classList.add('opensfm-file-select-category');
    span.textContent = category;
    return span;
  }

  _createSelecter(classNames) {
    const document = window.document;
    const span = document.createElement('span');
    span.classList.add('opensfm-file-select-text');
    span.textContent = 'Select files';

    const selecter = document.createElement('div');
    selecter.classList.add(
      'opensfm-file-select',
      'opensfm-file-select-inactive',
      ...classNames,
    );
    selecter.appendChild(span);

    return selecter;
  }

  _createSubmit() {
    const document = window.document;
    const span = document.createElement('span');
    span.classList.add('opensfm-file-select-submit');
    span.textContent = 'Submit';
    span.addEventListener('click', this._onSubmitClick);
    return span;
  }

  async _fetchList() {
    const options = {
      method: 'GET',
      headers: {
        Accept: 'application/json',
        'Content-Type': 'application/json',
      },
    };

    try {
      const result = await fetch(this._itemsUrl, options).then(r => r.json());
      if (result.items.length) {
        this._selecter.appendChild(this._submit);
      }
      const groups = new Map();
      for (const item of result.items) {
        const key = item.type;
        if (!groups.has(key)) {
          groups.set(key, []);
        }
        groups.get(key).push(item);
      }

      groups.forEach((items, category) => {
        const header = this._createCategoryHeader(category);
        this._selecter.appendChild(header);
        for (const item of items) {
          const container = this._createItemContainer(item);
          const selected = false;
          const key = item.name;
          const value = Object.assign({container, selected}, item);
          this._items.set(key, value);
          this._selecter.appendChild(value.container);
        }
      });
    } catch (error) {
      console.error(error);
    }
  }

  _onItemClick = event => {
    const name = event.target.textContent;
    const item = this._items.get(name);
    const selected = !item.selected;
    select(item, selected);
    if (item.selected) {
      item.container.classList.add('opensfm-file-select-selected');
    } else {
      item.container.classList.remove('opensfm-file-select-selected');
    }
  };

  _onSubmitClick = async event => {
    this._selecter.classList.add('opensfm-file-select-inactive');
    const selected = [];
    for (const item of traverse(this._items.values())) {
      if (item.selected && !!item.url) {
        selected.push(item);
      }
    }
    const options = {
      method: 'GET',
      headers: {
        Accept: 'application/json',
        'Content-Type': 'application/json',
      },
    };

    const items = selected.map(item =>
      fetch(item.url, options)
        .then(r => r.json())
        .then(data => Object.assign({data}, item)),
    );
    this._resolver(items);
    this._clear();
  };

  _verifyKind(kind) {
    if (kind !== 'file') {
      console.warn(`Unrecognized format ${kind}`);
      return false;
    }
    return true;
  }
}
