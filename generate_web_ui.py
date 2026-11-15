"""
Pre-build script: minify JavaScript, compact HTML and gzip into a build header.
"""

import gzip
import importlib.util
import json
import os
from pathlib import Path
import re
import zlib

try:
    Import("env")
except NameError:
    env = None

# SCons executes extra scripts without defining __file__.
PROJECT_DIR = (os.path.abspath(env.get("PROJECT_DIR", ".")) if env is not None
               else os.path.dirname(os.path.abspath(__file__)))
RJS_MIN_PATH = os.path.join(PROJECT_DIR, "third_party", "rjsmin", "rjsmin.py")


def load_js_minifier():
    spec = importlib.util.spec_from_file_location("airbridge_rjsmin", RJS_MIN_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load JavaScript minifier: {RJS_MIN_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if module.__version__ != "1.2.5":
        raise RuntimeError(f"unsupported rjsmin version {module.__version__}; expected 1.2.5")
    return module._make_jsmin(python_only=True)


js_minify = load_js_minifier()


def load_catalogs(project_dir=PROJECT_DIR):
    catalogs = {}
    for path in sorted(Path(project_dir, 'www', 'i18n').glob('*.json')):
        if len(path.stem) > 15 or not re.fullmatch(r'[a-z]{2}(?:-[A-Za-z0-9]{2,8})*', path.stem):
            raise ValueError(f'invalid language tag: {path.stem}')
        catalog = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(catalog, dict):
            raise ValueError(f'{path.stem}: catalog must be an object')
        for key, value in catalog.items():
            variants = list(value.values()) if isinstance(value, dict) else [value]
            if not variants or any(not isinstance(text, str) for text in variants):
                raise ValueError(f'{path.stem}: invalid message for {key}')
            if isinstance(value, dict) and ('other' not in value or not
                    set(value).issubset({'zero', 'one', 'two', 'few', 'many', 'other'})):
                raise ValueError(f'{path.stem}: invalid plural variants for {key}')
        catalogs[path.stem] = catalog
    if 'en' not in catalogs:
        raise ValueError('English UI catalog is required')
    base = catalogs['en']
    for language, catalog in catalogs.items():
        if catalog.keys() != base.keys():
            raise ValueError(f'{language}: catalog keys differ from English')
        for key, value in catalog.items():
            variants = list(value.values()) if isinstance(value, dict) else [value]
            reference = base[key]
            if isinstance(reference, dict) != isinstance(value, dict):
                raise ValueError(f'{language}: plural shape differs for {key}')
            references = list(reference.values()) if isinstance(reference, dict) else [reference]
            parameters = set(re.findall(r'\{(\w+)\}', references[0]))
            for text in variants + references:
                if not isinstance(text, str) or set(re.findall(r'\{(\w+)\}', text)) != parameters:
                    raise ValueError(f'{language}: invalid parameters for {key}')
    return catalogs


def load_themes(project_dir=PROJECT_DIR):
    themes = []
    properties = {}
    for path in sorted(Path(project_dir, 'www', 'themes').glob('*.css')):
        theme_id = path.stem
        if len(theme_id) > 31 or not re.fullmatch(r'[a-z][a-z0-9-]*', theme_id) or theme_id == 'auto':
            raise ValueError(f'invalid theme ID: {theme_id}')
        css = path.read_bytes()
        declarations = re.sub(r'/\*.*?\*/', '', css.decode('utf-8'), flags=re.DOTALL)
        schemes = re.findall(r'(?<![\w-])color-scheme\s*:\s*([^;}]+)', declarations)
        if len(schemes) != 1 or schemes[0].strip() not in ('light', 'dark'):
            raise ValueError(f'{theme_id}: expected one light or dark color-scheme')
        scheme = schemes[0].strip()
        properties[theme_id] = set(re.findall(r'(--[\w-]+)\s*:', declarations))
        themes.append({'id': theme_id, 'scheme': scheme,
                       'url': f'/themes/{theme_id}.{zlib.crc32(css):08x}.css', 'css': css})
    for required in ('dark', 'light'):
        if required not in properties:
            raise ValueError(f'{required} UI theme is required')
        if next(theme['scheme'] for theme in themes if theme['id'] == required) != required:
            raise ValueError(f'{required}: color-scheme must be {required}')
    for theme in themes:
        missing = properties['dark'] - properties[theme['id']]
        if missing:
            raise ValueError(f"{theme['id']}: missing palette properties: {', '.join(sorted(missing))}")
    return themes


def fill_config_sections(html, source, catalogs=None, themes=None):
    if catalogs is None:
        catalogs = load_catalogs()
    if themes is None:
        themes = load_themes()
    # These two declarative C++ tables are shared with Config's HTTP filter.
    sections = {}
    rows = source.split('} config_sections[] = {', 1)[1].split('};', 1)[0]
    for row in rows.splitlines():
        if not row.strip() or row.strip() in ('#if AB_WIREGUARD_ENABLED', '#endif'):
            continue
        match = re.fullmatch(r'\s*\{Config::Section::(\w+), "([^"]+)", "([^"]+)"(?:, (false))?\},\s*', row)
        if not match:
            raise ValueError(f'unsupported config section: {row}')
        symbol, name, label, hidden = match.groups()
        sections[symbol] = None if hidden else {'id': name, 'label': label, 'keys': [], 'options': {}}

    rows = source.split('static const KVEntry kv_table[] = {', 1)[1].split('};', 1)[0]
    for row in rows.splitlines():
        if not row.strip() or row.strip() in ('#if AB_WIREGUARD_ENABLED', '#endif'):
            continue
        match = re.fullmatch(
            r'\s*KV_(\w+)\("([^"]+)", "[^"]+", [\w.]+, '
            r'(?:"(?:\\.|[^"\\])*"|[^,]+), (\w+)(.*?)\),\s*', row)
        if not match:
            raise ValueError(f'unsupported config field: {row}')
        kind, key, section, choices = match.groups()
        if sections[section] is None:
            continue
        sections[section]['keys'].append(key)
        if kind == 'SECRET':
            sections[section].setdefault('secrets', []).append(key)
        if kind == 'LANGUAGE':
            sections[section]['options'][key] = [['auto', 'Browser language']] + [
                [tag, catalog['language.name']] for tag, catalog in catalogs.items()]
        if kind == 'THEME':
            sections[section]['options'][key] = [['auto', 'Automatic']] + [
                [theme['id'], theme['id']] for theme in themes]
        if kind in ('BOOL', 'ENUM', 'WG_MODE'):
            labels = (json.loads('[' + choices.removeprefix(', ') + ']') if choices
                      else ['0=Disabled', '1=Enabled'] if kind == 'BOOL' else [])
            if not labels:
                raise ValueError(f'missing enum choices: {key}')
            sections[section]['options'][key] = [label.split('=', 1) for label in labels]

    marker = '/* CONFIG_SECTIONS */ []'
    if html.count(marker) != 1:
        raise ValueError('expected one Config sections placeholder')
    html = html.replace(marker, json.dumps([s for s in sections.values() if s is not None], separators=(',', ':')))
    html = html.replace('/* UI_CATALOGS */ {}',
                        json.dumps(catalogs, ensure_ascii=True, separators=(',', ':')).replace('<', '\\u003c'))
    catalog = [{key: theme[key] for key in ('id', 'scheme', 'url')} for theme in themes]
    return html.replace('/* UI_THEMES */ []', json.dumps(catalog, separators=(',', ':')))


def minify_html(html):
    # Regex minification corrupts JS strings, regex literals and line comments.
    # Minify JS separately and exclude raw-text blocks from HTML whitespace edits.
    blocks = re.split(
        r'(<!--.*?-->|<(?:script|style|pre|textarea)\b[^>]*>.*?</(?:script|style|pre|textarea)\s*>)',
        html, flags=re.DOTALL | re.IGNORECASE)
    for i, block in enumerate(blocks):
        if i % 2:
            if block.startswith('<!--'):
                blocks[i] = ''
            elif re.match(r'<script\b', block, re.IGNORECASE):
                opening = block.index('>') + 1
                closing = block.lower().rindex('</script')
                blocks[i] = block[:opening] + js_minify(block[opening:closing]) + block[closing:]
        else:
            block = re.sub(r'>\s+<', '><', block)
            blocks[i] = re.sub(r'\s+', ' ', block)
    return ''.join(blocks).strip()


def generate_header(project_dir, output_dir):
    html_path = os.path.join(project_dir, "www", "index.html")
    header_path = os.path.join(output_dir, "web_ui_generated.h")
    if not os.path.exists(html_path):
        print(f"[web_ui] WARNING: {html_path} not found")
        return
    raw = Path(html_path).read_text(encoding="utf-8")
    catalogs = load_catalogs(project_dir)
    themes = load_themes(project_dir)
    raw = fill_config_sections(raw, Path(project_dir, 'src', 'app_config.cpp').read_text(encoding='utf-8'),
                               catalogs, themes)
    language_header = Path(output_dir, 'ui_languages_generated.h')
    languages = ('// Auto-generated from www/i18n, do not edit!\n#pragma once\n'
                 'static constexpr const char *UI_LANGUAGE_TAGS[] = {' +
                 ', '.join(json.dumps(tag) for tag in catalogs) + '};\n')
    if not language_header.exists() or language_header.read_text(encoding='utf-8') != languages:
        language_header.parent.mkdir(parents=True, exist_ok=True)
        language_header.write_text(languages, encoding='utf-8')
    theme_header = Path(output_dir, 'ui_themes_generated.h')
    theme_ids = ('// Auto-generated from www/themes, do not edit!\n#pragma once\n'
                 'static constexpr const char *UI_THEME_IDS[] = {' +
                 ', '.join(json.dumps(theme['id']) for theme in themes) + '};\n')
    if not theme_header.exists() or theme_header.read_text(encoding='utf-8') != theme_ids:
        theme_header.parent.mkdir(parents=True, exist_ok=True)
        theme_header.write_text(theme_ids, encoding='utf-8')
    minified = minify_html(raw)
    compressed = gzip.compress(minified.encode("utf-8"), compresslevel=9, mtime=0)
    lines = [
        "// Auto-generated from www/index.html, do not edit!\n",
        "#pragma once\n",
        "#include <stdint.h>\n",
        "#include <stddef.h>\n\n",
        f"#define HTML_PAGE_GZ_SIZE {len(compressed)}\n\n",
    ]
    assets = [('HTML_PAGE_GZ', compressed)] + [
        (f'UI_THEME_{index}_GZ', gzip.compress(theme['css'], compresslevel=9, mtime=0))
        for index, theme in enumerate(themes)]
    for name, payload in assets:
        lines.append(f'static const uint8_t {name}[] PROGMEM = {{\n')
        for offset in range(0, len(payload), 16):
            lines.append('    ' + ', '.join(f'0x{byte:02x}' for byte in payload[offset:offset+16]) + ',\n')
        lines.append('};\n')
    lines.append('\nstruct UiThemeAsset { const char *url; const uint8_t *data; size_t size; };\n'
                 'static const UiThemeAsset UI_THEME_ASSETS[] = {\n')
    for index, theme in enumerate(themes):
        lines.append(f'    {{"{theme["url"]}", UI_THEME_{index}_GZ, sizeof(UI_THEME_{index}_GZ)}},\n')
    lines.append('};\n')
    content = "".join(lines)
    header = Path(header_path)
    if not header.exists() or header.read_text(encoding="utf-8") != content:
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text(content, encoding="utf-8")
        print(f"[web_ui] {html_path} ({len(raw)} -> {len(minified)} minified -> {len(compressed)} gzipped)")


if env is not None:
    output_dir = os.path.join(env.subst("$BUILD_DIR"), "generated")
    generate_header(PROJECT_DIR, output_dir)
    env.AppendUnique(CPPPATH=[output_dir])
elif __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", help="Directory for the generated header")
    generate_header(PROJECT_DIR, parser.parse_args().output_dir)
