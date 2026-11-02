# -*- coding: utf-8 -*-
"""
渲染《代码实现说明》Word 文档。
内容来自 tools/code_doc_content.txt（纯文本描述），代码片段来自 _snippets.json。
这样做的好处：正文可含任意引号，不会与 Python 字符串语法冲突。
"""
import io, os, json
from docx import Document
from docx.shared import Pt, RGBColor, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONTENT = os.path.join(HERE, 'code_doc_content.txt')
SNIP = os.path.join(HERE, 'code_snippets.json')
OUT = os.path.join(ROOT, '文档', 'Balance_Ctrl_代码实现说明.docx')

S = json.load(io.open(SNIP, encoding='utf-8'))


def setf(run, name, size, bold=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.font.bold = bold
    if color is not None:
        run.font.color.rgb = color
    run._element.rPr.rFonts.set(qn('w:eastAsia'), name)


def build():
    doc = Document()
    st = doc.styles['Normal']
    st.font.name = '宋体'
    st.font.size = Pt(10.5)
    st.element.rPr.rFonts.set(qn('w:eastAsia'), '宋体')

    lines = io.open(CONTENT, encoding='utf-8').read().split('\n')
    i = 0
    n_snip = 0
    while i < len(lines):
        ln = lines[i].rstrip('\r')
        i += 1

        if ln.startswith('#COVER') and not ln.startswith('#COVERINFO'):
            for _ in range(3):
                doc.add_paragraph()
            for _k in range(3):
                t, sz = lines[i].split('|')
                i += 1
                p = doc.add_paragraph()
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                setf(p.add_run(t), '黑体', int(sz), True, RGBColor(0x1F, 0x3B, 0x73))
            continue

        if ln.startswith('#COVERINFO'):
            t = doc.add_table(rows=1, cols=2)
            t.style = 'Light Grid Accent 1'
            t.alignment = WD_TABLE_ALIGNMENT.CENTER
            for c, h in zip(t.rows[0].cells, ['项目', '内容']):
                c.text = ''
                setf(c.paragraphs[0].add_run(h), '黑体', 9.5, True)
            while i < len(lines) and not lines[i].startswith('#'):
                r = lines[i].split('|')
                i += 1
                if len(r) < 2:
                    continue
                cs = t.add_row().cells
                for c, v in zip(cs, r):
                    c.text = ''
                    setf(c.paragraphs[0].add_run(v), '宋体', 9)
            doc.add_paragraph()
            continue

        if ln.startswith('#H1 '):
            p = doc.add_heading(level=1)
            setf(p.add_run(ln[4:]), '黑体', 16, True, RGBColor(0x1F, 0x3B, 0x73))
            continue
        if ln.startswith('#H2 '):
            p = doc.add_heading(level=2)
            setf(p.add_run(ln[4:]), '黑体', 13, True, RGBColor(0x1F, 0x3B, 0x73))
            continue
        if ln.startswith('#H3 '):
            p = doc.add_heading(level=3)
            setf(p.add_run(ln[4:]), '黑体', 11.5, True, RGBColor(0x33, 0x33, 0x33))
            continue
        if ln.startswith('#P '):
            p = doc.add_paragraph()
            p.paragraph_format.first_line_indent = Cm(0.74)
            setf(p.add_run(ln[3:]), '宋体', 10.5)
            continue
        if ln.startswith('#B '):
            p = doc.add_paragraph(style='List Bullet')
            setf(p.add_run(ln[3:]), '宋体', 10.5)
            continue
        if ln.startswith('#N '):
            p = doc.add_paragraph()
            p.paragraph_format.left_indent = Cm(0.6)
            setf(p.add_run(ln[3:]), '楷体', 10, False, RGBColor(0xA0, 0x30, 0x00))
            continue
        if ln.startswith('#PB'):
            doc.add_page_break()
            continue
        if ln.startswith('#S '):
            key = ln[3:].strip()
            if key not in S:
                raise SystemExit('缺少代码片段: ' + key)
            n_snip += 1
            for cl in S[key].split('\n'):
                p = doc.add_paragraph()
                p.paragraph_format.left_indent = Cm(0.6)
                p.paragraph_format.space_after = Pt(0)
                p.paragraph_format.space_before = Pt(0)
                setf(p.add_run(cl if cl else ' '), 'Consolas', 8.5)
            doc.add_paragraph()
            continue
        if ln.startswith('#C'):
            while i < len(lines) and not lines[i].startswith('#/C'):
                p = doc.add_paragraph()
                p.paragraph_format.left_indent = Cm(0.6)
                p.paragraph_format.space_after = Pt(0)
                p.paragraph_format.space_before = Pt(0)
                setf(p.add_run(lines[i] if lines[i] else ' '), 'Consolas', 8.5)
                i += 1
            i += 1
            doc.add_paragraph()
            continue
        if ln.startswith('#T '):
            hdr = ln[3:].split('|')
            t = doc.add_table(rows=1, cols=len(hdr))
            t.style = 'Light Grid Accent 1'
            t.alignment = WD_TABLE_ALIGNMENT.CENTER
            for c, h in zip(t.rows[0].cells, hdr):
                c.text = ''
                setf(c.paragraphs[0].add_run(h), '黑体', 9.5, True)
            while i < len(lines) and not lines[i].startswith('#/T'):
                r = lines[i].split('|')
                i += 1
                cs = t.add_row().cells
                for k, v in enumerate(r[:len(hdr)]):
                    cs[k].text = ''
                    setf(cs[k].paragraphs[0].add_run(v), '宋体', 9)
            i += 1
            doc.add_paragraph()
            continue

    # ---- 附录：代码行数统计（脚本自动读取，不手写）----
    src = os.path.join(ROOT, 'src')
    files = sorted(f for f in os.listdir(src) if f.endswith('.v'))
    tot = 0
    doc.add_page_break()
    p = doc.add_heading(level=1)
    setf(p.add_run('附录  代码规模统计（脚本自动读取）'), '黑体', 16, True, RGBColor(0x1F, 0x3B, 0x73))
    t = doc.add_table(rows=1, cols=2)
    t.style = 'Light Grid Accent 1'
    for c, h in zip(t.rows[0].cells, ['文件', '行数']):
        c.text = ''
        setf(c.paragraphs[0].add_run(h), '黑体', 9.5, True)
    for f in files:
        n = sum(1 for _ in io.open(os.path.join(src, f), encoding='utf-8'))
        tot += n
        cs = t.add_row().cells
        cs[0].text = ''
        setf(cs[0].paragraphs[0].add_run(f), '宋体', 9)
        cs[1].text = ''
        setf(cs[1].paragraphs[0].add_run(str(n)), '宋体', 9)
    cs = t.add_row().cells
    cs[0].text = ''
    setf(cs[0].paragraphs[0].add_run('合计 %d 个文件' % len(files)), '黑体', 9, True)
    cs[1].text = ''
    setf(cs[1].paragraphs[0].add_run(str(tot)), '黑体', 9, True)

    doc.save(OUT)
    print('已生成：%s' % OUT)
    print('插入代码片段 %d 段；RTL %d 个文件 / %d 行' % (n_snip, len(files), tot))


if __name__ == '__main__':
    build()
