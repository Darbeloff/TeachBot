/**
 * Sleep for some milliseconds.
 *
 * Waits for a Promise to resolve after a given amount of time in milliseconds.
 * Usage: await sleep(ms);
 *
 * @param {number}	ms	Time to sleep in milliseconds.
 *
 * @return {object}	The Promise to be resolved after ms milliseconds.
 */
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}