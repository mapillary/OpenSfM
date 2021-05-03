/**
 * @format
 */

export function pixelToViewport(pixelPoint, container) {
  const canvasWidth = container.offsetWidth;
  const canvasHeight = container.offsetHeight;
  const viewportX = (2 * pixelPoint[0]) / canvasWidth - 1;
  const viewportY = 1 - (2 * pixelPoint[1]) / canvasHeight;
  return [viewportX, viewportY];
}
