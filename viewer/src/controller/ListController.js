/**
 * @format
 */

export class ListController {
  constructor(options) {
    this._eventType = options.eventType;
    this._emitter = options.emitter;

    this._config = {
      items: {},
      toggle: () => this._onToggle(),
    };

    this._folder = options.folder;
    this._toggle = options.folder.add(this._config, 'toggle');
    this._setToggleText();
  }

  addItems(ids) {
    const items = this._config.items;
    const folder = this._folder;
    for (const id of ids) {
      if (id in items) {
        throw new Error(`Item exists ${id}`);
      }
      items[id] = true;
      folder
        .add(items, id)
        .listen()
        .onChange(() => this._onChange());
    }
    this._onChange();
  }

  _checkAllActive() {
    const items = this._config.items;
    const ids = Object.keys(items);
    return (
      ids.length &&
      ids.map(id => items[id]).reduce((acc, val) => acc && val, true)
    );
  }

  _onChange() {
    this._setToggleText();
    const items = this._config.items;
    const active = Object.keys(items).filter(id => items[id]);
    const type = this._eventType;
    this._emitter.fire(type, {active, type});
  }

  _onToggle() {
    const items = this._config.items;
    const all = this._checkAllActive(items);
    for (const id of Object.keys(items)) {
      items[id] = !all;
    }
    this._onChange();
  }

  _setToggleText() {
    const all = this._checkAllActive(this._items);
    if (all) {
      this._toggle.name('Hide all');
    } else {
      this._toggle.name('Show all');
    }
  }
}
