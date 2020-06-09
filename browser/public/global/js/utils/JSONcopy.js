/**
 * Copy an object.
 *
 * Uses JSON.stringify and JSON.parse to perform a deep copy of an object.
 * Will not copy object methods or circular objects.
 *
 * @param {object}	obj	An object to copy.
 *
 * @return {object}	The copied object.
 */
function JSONcopy(obj) {
	return JSON.parse(JSON.stringify(obj));
}