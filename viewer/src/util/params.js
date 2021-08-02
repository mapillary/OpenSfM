/**
 * @format
 */

export function parseHash(hash) {
  if (!hash) {
    return {};
  }
  const hashContent = hash.substring(1);
  const pl = /\+/g, // Regex for replacing addition symbol with a space
    search = /([^&=]+)=?([^&]*)/g,
    decode = s => decodeURIComponent(s.replace(pl, ' ')),
    params = {};

  let match;
  while ((match = search.exec(hashContent))) {
    params[decode(match[1])] = decode(match[2]);
  }
  for (const param of Object.keys(params)) {
    const split = params[param].split(',');
    params[param] = split.length > 1 ? split : params[param];
  }
  return params;
}

export function createProviderOptions(params) {
  const location = window.location;
  const options = {
    endpoint: `${location.protocol}//${location.host}`,
    imagePath: 'image',
    reconstructionPaths: null,
  };
  if (!!params.file) {
    if (params.file instanceof Array) {
      options.reconstructionPaths = params.file;
    } else {
      options.reconstructionPaths = [params.file];
      // fallback image based on file
      options.imagePath = `${params.file.replace(/[^/]*$/, '')}${'image'}`;
    }
  }
  if (!!params.image) {
    // image takes precedence over fallback image
    options.imagePath = params.image;
  }
  return options;
}
