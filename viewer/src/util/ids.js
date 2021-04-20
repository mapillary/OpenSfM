/**
 * Invent unique IDs to be able to load
 * reconstructions with the same image IDs.
 *
 * @format
 */

export function imageToUniqueID(image) {
  return `${image.clusterId}|${image.id}`;
}

export function imageEntToUniqueID(imageEnt) {
  return `${imageEnt.cluster.id}|${imageEnt.id}`;
}

function uniqueIDToIDs(uniqueId) {
  return uniqueId.split('|');
}

export function uniqueIDToClusterID(uniqueId) {
  return uniqueIDToIDs(uniqueId)[0];
}

export function uniqueIDToImageID(uniqueId) {
  return uniqueIDToIDs(uniqueId)[1];
}
