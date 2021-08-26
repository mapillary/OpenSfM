/**
 * Formats a string, replacing designated variable names with their values
 * e.g. "Hello {name}" and { name: "Bob" } returns "Hello Bob"
 *      "Vec3: [{x}, {y}, {z}]" { x:1, y:2, z:3 } returns "Vec3: [1, 2, 3]"
 */
export function formatString(fstring, args) {
    var str = fstring;
    for (const key in args) {
        str = str.replace(new RegExp("\\{" + key + "\\}", "gi"), args[key]);
    }
    return str;
}
