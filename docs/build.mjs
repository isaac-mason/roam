import fs from 'node:fs';
import path from 'node:path';

const docsDir = path.dirname(new URL(import.meta.url).pathname);
const projectRoot = path.join(docsDir, '..');
const readmeTemplatePath = path.join(docsDir, 'README.template.md');
const readmeOutPath = path.join(projectRoot, 'README.md');
const examplesJsonPath = path.join(projectRoot, 'demos/examples.json');

const WIDTH = 100;
const DEMOS_URL = 'https://isaac-mason.github.io/roam/';
const check = process.argv.includes('--check');

let failed = false;
function fail(message) {
    console.error(message);
    failed = true;
}

let readmeText = fs.readFileSync(readmeTemplatePath, 'utf-8');

const examplesData = JSON.parse(fs.readFileSync(examplesJsonPath, 'utf-8'));

function exampleCell(id, width, height, strong) {
    const example = examplesData[id];
    const title = example.title || id;
    const label = strong ? `<strong>${title}</strong>` : title;
    return `    <td align="center">
      <a href="${DEMOS_URL}?example=${id}">
        <img src="./demos/screenshots/${id}.png" width="${width}" height="${height}" style="object-fit:cover;"/><br/>
        ${label}
      </a><br/>
      <sub><a href="./demos/examples/${id}.cpp">source</a></sub>
    </td>`;
}

/* <TOC /> */
const tocLines = [];
let inFence = false;
for (const line of readmeText.split('\n')) {
    if (line.startsWith('```')) inFence = !inFence;
    if (inFence) continue;
    const match = line.match(/^(#{2,6})\s+(.*)$/);
    if (!match) continue;
    const level = match[1].length - 1;
    const title = match[2].trim();
    if (title === 'Table of Contents') continue;
    const anchor = title
        .toLowerCase()
        .replace(/[^\w\s-]/g, '')
        .replace(/\s/g, '-');
    tocLines.push(`${'  '.repeat(level - 1)}- [${title}](#${anchor})`);
}
readmeText = readmeText.replace(/<TOC\s*\/>/g, tocLines.join('\n'));

/* <Examples /> */
readmeText = readmeText.replace(/<Examples\s*\/>/g, () => {
    const ids = Object.keys(examplesData);
    const columns = 3;
    let html = '<table>\n';
    for (let i = 0; i < ids.length; i += columns) {
        html += '  <tr>\n';
        html += ids
            .slice(i, i + columns)
            .map((id) => exampleCell(id, 180, 120, false))
            .join('\n');
        html += '\n  </tr>\n';
    }
    html += '</table>';
    return html;
});

/* <ExamplesTable ids="example_find_path,example_find_smooth_path" /> */
readmeText = readmeText.replace(/<ExamplesTable\s+ids=["'](.+?)["']\s*\/>/g, (fullMatch, idsText) => {
    const ids = idsText.split(',').map((id) => id.trim());
    for (const id of ids) {
        if (!examplesData[id]) fail(`example '${id}' not found in demos/examples.json`);
    }
    const cells = ids
        .filter((id) => examplesData[id])
        .map((id) => exampleCell(id, 200, 133, true))
        .join('\n');
    return `<table>\n  <tr>\n${cells}\n  </tr>\n</table>`;
});

/* <Snippet source="./snippets/file.cpp" select="group" /> */
readmeText = readmeText.replace(/<Snippet\s+source=["'](.+?)["']\s+select=["'](.+?)["']\s*\/>/g, (fullMatch, sourcePath, groupName) => {
    const absSourcePath = path.join(docsDir, sourcePath);
    if (!fs.existsSync(absSourcePath)) {
        fail(`snippet source not found: ${sourcePath}`);
        return fullMatch;
    }
    const sourceText = fs.readFileSync(absSourcePath, 'utf-8');

    const groupRegex = new RegExp(
        String.raw`^([ \t]*)\/\*[ \t]*SNIPPET_START:[ \t]*${groupName}[ \t]*\*\/[\r\n]+([\s\S]*?)^[ \t]*\/\*[ \t]*SNIPPET_END:[ \t]*${groupName}[ \t]*\*\/`,
        'gm',
    );
    const matches = Array.from(sourceText.matchAll(groupRegex));
    if (matches.length === 0) {
        fail(`snippet '${groupName}' not found in ${sourcePath}`);
        return fullMatch;
    }

    const parts = matches.map((match) => {
        const baseIndent = match[1] || '';
        let code = match[2];
        if (baseIndent) code = code.replace(new RegExp(`^${baseIndent}`, 'gm'), '');
        code = code.replace(/^.*\/\*[ \t]*SNIPPET_(START|END):[^*]*\*\/.*\n?/gm, '');
        return code;
    });

    const code = parts.join('').replace(/^\s*\n|\n\s*$/g, '');
    for (const line of code.split('\n')) {
        if (line.length > WIDTH) fail(`snippet '${groupName}' in ${sourcePath} has a line over ${WIDTH} columns:\n${line}`);
    }
    return '```cpp\n' + code + '\n```';
});

/* <Declarations of="roam/generate/heightfield.h#create roam/debug/debug.h#create_heightfield_helper" /> */
readmeText = readmeText.replace(/<Declarations\s+of=["']([^"']+)["']\s*\/>/g, (fullMatch, refsText) => {
    const refs = refsText.trim().split(/\s+/);
    const declarations = [];
    for (const ref of refs) {
        const [header, name] = ref.split('#');
        const declaration = findDeclaration(header, name);
        if (!declaration) {
            fail(`declaration '${name}' not found in ${header}`);
            continue;
        }
        declarations.push(declaration);
    }
    const separator = declarations.some((declaration) => declaration.includes('\n')) ? '\n\n' : '\n';
    return '```cpp\n' + declarations.join(separator) + '\n```';
});

const leftover = readmeText.match(/<(TOC|Examples|ExamplesTable|Snippet|Declarations)\b[^>]*\/>/);
if (leftover) fail(`unexpanded tag: ${leftover[0]}`);

if (failed) process.exit(1);

if (check) {
    const current = fs.existsSync(readmeOutPath) ? fs.readFileSync(readmeOutPath, 'utf-8') : '';
    if (current !== readmeText) {
        console.error('README.md is out of date: run `node docs/build.mjs`');
        process.exit(1);
    }
} else {
    fs.writeFileSync(readmeOutPath, readmeText, 'utf-8');
}

/* utils */

function findDeclaration(header, name) {
    const headerPath = path.join(projectRoot, header);
    if (!fs.existsSync(headerPath)) return null;
    const lines = fs.readFileSync(headerPath, 'utf-8').split('\n');

    const namespaces = [];
    let depth = 0;
    for (let i = 0; i < lines.length; i++) {
        const line = lines[i];
        const namespaceMatch = line.match(/^namespace (\w+) \{$/);
        if (namespaceMatch) {
            depth++;
            namespaces.push({ name: namespaceMatch[1], depth });
            continue;
        }
        const atTop = namespaces.length > 0 && namespaces[namespaces.length - 1].depth === depth;
        const opened = (line.match(/\{/g) || []).length;
        const closed = (line.match(/\}/g) || []).length;
        if (!atTop) {
            depth += opened - closed;
            while (namespaces.length > 0 && depth < namespaces[namespaces.length - 1].depth) namespaces.pop();
            continue;
        }

        const module = namespaces[namespaces.length - 1].name;

        if (new RegExp(String.raw`^struct ${name} \{$`).test(line)) {
            const body = [];
            for (let j = i; j < lines.length; j++) {
                body.push(lines[j]);
                if (lines[j] === '};') break;
            }
            return body.join('\n');
        }

        const functionMatch = line.match(new RegExp(String.raw`^(\S[^;{}()]*?\b)${name}\(`));
        if (functionMatch && module && module !== 'roam') {
            let text = '';
            for (let j = i; j < lines.length; j++) {
                text += ' ' + lines[j].trim();
                if (lines[j].includes(';')) break;
            }
            text = text.trim().replace(/\s+/g, ' ');
            const qualified = text.replace(new RegExp(String.raw`\b${name}\(`), `${module}::${name}(`);
            return wrapDeclaration(qualified);
        }

        depth += opened - closed;
        while (namespaces.length > 0 && depth < namespaces[namespaces.length - 1].depth) namespaces.pop();
    }
    return null;
}

function wrapDeclaration(text) {
    if (text.length <= WIDTH) return text;
    const open = text.indexOf('(');
    const close = text.lastIndexOf(')');
    const head = text.slice(0, open + 1);
    const tail = text.slice(close);
    const args = splitArguments(text.slice(open + 1, close));

    const indent = '    ';
    const out = [head];
    let current = indent;
    args.forEach((arg, index) => {
        const piece = arg + (index < args.length - 1 ? ',' : '');
        if (current.trim() && current.length + 1 + piece.length > WIDTH) {
            out.push(current);
            current = indent;
        }
        current += (current.trim() ? ' ' : '') + piece;
    });
    out.push(current + tail);
    return out.join('\n');
}

function splitArguments(text) {
    const args = [];
    let depth = 0;
    let current = '';
    for (const ch of text) {
        if ('({['.includes(ch)) depth++;
        if (')}]'.includes(ch)) depth--;
        if (ch === ',' && depth === 0) {
            args.push(current.trim());
            current = '';
        } else {
            current += ch;
        }
    }
    if (current.trim()) args.push(current.trim());
    return args;
}
