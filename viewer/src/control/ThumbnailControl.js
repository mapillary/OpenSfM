/**
 * @format
 */

import {uniqueIDToImageID} from '../util/ids.js';
import {Copier} from './Copier.js';

export class ThumbnailControl {
  constructor(options) {
    this._imageText = this._createText('Image: ');
    this._clusterText = this._createText('Cluster: ');
    this._clusterURL = this._createText('URL: ');
    this._thumb = this._createThumb();
    const container = this._createContainer();
    container.appendChild(this._thumb);
    container.appendChild(this._imageText.element);
    container.appendChild(this._clusterText.element);
    container.appendChild(this._clusterURL.element);
    this._container = container;
    this._visible = options.visible;
    this._provider = options.provider;
    this._image = null;
  }

  get container() {
    return this._container;
  }

  hide() {
    if (!this._visible) {
      return;
    }
    this._container.classList.add('opensfm-hidden');
    this._visible = false;
  }

  update(image) {
    this._image = image;
    this._render();
  }

  show() {
    if (this._visible) {
      return;
    }
    this._container.classList.remove('opensfm-hidden');
    this._visible = true;
    this._render();
  }

  _createContainer() {
    const header = document.createElement('span');
    header.classList.add('opensfm-info-text', 'opensfm-info-text-header');
    header.textContent = 'Thumbnail';

    const container = document.createElement('div');
    container.classList.add('opensfm-control-container', 'opensfm-hidden');
    container.appendChild(header);
    return container;
  }

  _createText(prefix) {
    const document = window.document;
    const element = document.createElement('span');
    element.classList.add('opensfm-info-text', 'opensfm-info-inline');
    const copier = new Copier({container: element, content: null});
    return {copier, element, prefix};
  }

  _createThumb() {
    const thumb = document.createElement('img');
    thumb.classList.add('opensfm-thumb');
    return thumb;
  }

  _setTextContent(textItem, content) {
    textItem.element.textContent = textItem.prefix + content;
    textItem.copier.setContent(content);
  }

  _render() {
    if (!this._image || !this._visible) {
      return;
    }
    const image = this._image;
    this._thumb.src = image.image.src;
    const clusterId = image.clusterId;
    const clusterURL = this._provider.rawData[clusterId].file.url;
    const imageId = uniqueIDToImageID(image.id);
    this._setTextContent(this._clusterURL, clusterURL);
    this._setTextContent(this._clusterText, clusterId);
    this._setTextContent(this._imageText, imageId);
  }
}
