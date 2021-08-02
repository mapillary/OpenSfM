/**
 * @format
 */

export class StatsControl {
  constructor(options) {
    const container = this._createContainer();
    this._container = container;
    this._provider = options.provider;
    this._shotCount = 0;
    this._pointCount = 0;
    this._total = this._createText(
      this._makeContent('Total', this._shotCount, this._pointCount),
    );
    this._container.appendChild(this._total);
    this._visible = false;
    if (options.visible) {
      this.show();
    }
  }

  get container() {
    return this._container;
  }

  addRawData(rawData) {
    for (const data of Object.values(rawData)) {
      const id = data.id;
      const url = data.file.url;
      const cluster = data.cluster;
      const shotCount = Object.keys(cluster.shots).length;
      const pointCount = Object.keys(cluster.points).length;
      const content = this._makeContent(id, shotCount, pointCount, url);
      this.addStatRow(content);

      this._shotCount += shotCount;
      this._pointCount += pointCount;
    }

    this._total.textContent = this._makeContent(
      'Total',
      this._shotCount,
      this._pointCount,
    );
  }

  addStatRow(content) {
    const stat = this._createText(content);
    stat.classList.add('opensfm-info-text-stat');
    this._container.appendChild(stat);
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
    header.textContent = 'Stats';

    const container = document.createElement('div');
    container.classList.add('opensfm-control-container', 'opensfm-hidden');
    container.appendChild(header);
    return container;
  }

  _createText(content) {
    const document = window.document;
    const element = document.createElement('span');
    element.classList.add('opensfm-info-text');
    element.textContent = content;
    return element;
  }

  _makeContent(prefix, shotCount, pointCount, suffix) {
    const append = suffix ? ` (${suffix})` : '';
    return `${prefix}: ${shotCount} images, ${pointCount} points${append}`;
  }
}
