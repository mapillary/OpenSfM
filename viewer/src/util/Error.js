/**
 * @format
 */

export class CancelledError extends Error {
  constructor(message) {
    super(message);
    Object.setPrototypeOf(this, CancelledError.prototype);
    this.name = 'CancelledError';
  }
}
