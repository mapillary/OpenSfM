/**
 * @format
 */

export function isReconstructionData(data) {
  return (
    data instanceof Array &&
    data.length > 0 &&
    !!data[0].cameras &&
    !!data[0].shots
  );
}
