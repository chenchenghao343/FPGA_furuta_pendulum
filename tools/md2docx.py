# -*- coding: utf-8 -*-
"""md2docx.py -- 把本工程的报告 Markdown 转成排版好的 Word。

只处理本工程文档实际用到的语法：
  # / ## / ### / ####  标题
  > 引用块（作为文档信息块）
  ``` 代码块 ```（Consolas + 灰底，不缩进）
  | 表格 |（Table Grid，首行加粗）
  - 无序列表 / 1. 有序列表
  **粗体**、`行内代码`
  --- 分隔线

中文排版：正文宋体五号、1.5 倍行距、首行缩进 2 字符；标题黑体。
"""
import re
import sys
import io
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement


BODY_FONT_CN = "宋体"
BODY_FONT_EN = "Times New Roman"
HEAD_FONT_CN = "黑体"
CODE_FONT = "Consolas"


def set_run_font(run, cn=BODY_FONT_CN, en=BODY_FONT_EN, size=None, bold=None,
                 color=None):
    run.font.name = en
    rpr = run._element.get_or_add_rPr()
    rf = rpr.find(qn("w:rFonts"))
    if rf is None:
        rf = OxmlElement("w:rFonts")
        rpr.append(rf)
    rf.set(qn("w:ascii"), en)
    rf.set(qn("w:hAnsi"), en)
    rf.set(qn("w:eastAsia"), cn)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if color is not None:
        run.font.color.rgb = color


def shade_paragraph(p, fill):
    pPr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    pPr.append(shd)


def shade_cell(cell, fill):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    tcPr.append(shd)


def set_repeat_header(row):
    trPr = row._tr.get_or_add_trPr()
    h = OxmlElement("w:tblHeader")
    h.set(qn("w:val"), "true")
    trPr.append(h)


def set_cant_split(row):
    trPr = row._tr.get_or_add_trPr()
    c = OxmlElement("w:cantSplit")
    c.set(qn("w:val"), "true")
    trPr.append(c)


INLINE_RE = re.compile(r"(\*\*.+?\*\*|`[^`]+`)")


def _emit(p, text, size, cn, en, bold, color):
    """把一段文本写进段落；段内按 `反引号` 切出代码片段。"""
    for part in re.split(r"(`[^`]+`)", text):
        if not part:
            continue
        if part.startswith("`") and part.endswith("`") and len(part) > 2:
            r = p.add_run(part[1:-1])
            set_run_font(r, BODY_FONT_CN, CODE_FONT, size - 0.5, bold)
            r.font.color.rgb = RGBColor(0xC0, 0x30, 0x30)
        else:
            r = p.add_run(part)
            set_run_font(r, cn, en, size, bold, color)


def add_inline(p, text, size=10.5, cn=BODY_FONT_CN, en=BODY_FONT_EN,
               base_bold=False, color=None):
    """把含 **粗体** / `代码` 的文本写进段落（支持粗体里套代码）。"""
    for seg in re.split(r"(\*\*.+?\*\*)", text):
        if not seg:
            continue
        if seg.startswith("**") and seg.endswith("**") and len(seg) > 4:
            _emit(p, seg[2:-2], size, cn, en, True, color)
        else:
            _emit(p, seg, size, cn, en, base_bold, color)


def split_row(ln):
    """按 | 切表格行，但反引号内的 | 不切（如 `|phi|<130`）。"""
    s = ln.strip()
    if s.startswith("|"):
        s = s[1:]
    if s.endswith("|"):
        s = s[:-1]
    cells, cur, in_tick = [], [], False
    for ch in s:
        if ch == "`":
            in_tick = not in_tick
            cur.append(ch)
        elif ch == "|" and not in_tick:
            cells.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    cells.append("".join(cur).strip())
    return cells


def add_heading(doc, text, level):
    """level: 1=文档标题 2=章 3=节 4=小节"""
    p = doc.add_paragraph()
    sizes = {1: 20, 2: 16, 3: 13, 4: 11.5}
    before = {1: 0, 2: 18, 3: 12, 4: 8}
    after = {1: 6, 2: 10, 3: 6, 4: 4}
    p.paragraph_format.space_before = Pt(before[level])
    p.paragraph_format.space_after = Pt(after[level])
    p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
    if level == 1:
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    clean = text.replace("**", "").replace("`", "")
    r = p.add_run(clean)
    set_run_font(r, HEAD_FONT_CN, HEAD_FONT_CN, sizes[level], True)
    if level <= 2:
        r.font.color.rgb = RGBColor(0x1F, 0x3B, 0x63)
    return p


def add_body(doc, text, indent=True, size=10.5):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.line_spacing = 1.5
    pf.space_after = Pt(3)
    if indent:
        pf.first_line_indent = Pt(size * 2)      # 首行缩进 2 字符
    add_inline(p, text, size)
    return p


def add_code(doc, lines):
    for i, ln in enumerate(lines):
        p = doc.add_paragraph()
        pf = p.paragraph_format
        pf.line_spacing_rule = WD_LINE_SPACING.SINGLE
        pf.space_before = Pt(6 if i == 0 else 0)
        pf.space_after = Pt(6 if i == len(lines) - 1 else 0)
        pf.left_indent = Cm(0.6)
        shade_paragraph(p, "F3F3F3")
        r = p.add_run(ln if ln else " ")
        set_run_font(r, BODY_FONT_CN, CODE_FONT, 9)


def add_table(doc, rows):
    ncol = max(len(r) for r in rows)
    t = doc.add_table(rows=len(rows), cols=ncol)
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for ri, row in enumerate(rows):
        for ci in range(ncol):
            cell = t.cell(ri, ci)
            cell.text = ""
            txt = row[ci] if ci < len(row) else ""
            parts = txt.split("<br>")          # 单元格内换行
            for k, seg in enumerate(parts):
                p = cell.paragraphs[0] if k == 0 else cell.add_paragraph()
                p.paragraph_format.space_before = Pt(2)
                p.paragraph_format.space_after = Pt(2)
                p.paragraph_format.line_spacing = 1.15
                add_inline(p, seg, 9, base_bold=(ri == 0))
            if ri == 0:
                shade_cell(cell, "E8EEF7")
    # 首列纵向合并：空单元格与上一个非空单元格合并（表示"同一组"）
    if ncol >= 2 and len(rows) > 2:
        r = 1
        while r < len(rows):
            cur = rows[r][0].strip() if len(rows[r]) > 0 else ""
            prev = rows[r - 1][0].strip() if len(rows[r - 1]) > 0 else ""
            if cur == "" and prev != "":
                start, end = r - 1, r
                while end + 1 < len(rows) and (len(rows[end + 1]) == 0
                                               or rows[end + 1][0].strip() == ""):
                    end += 1
                try:
                    t.cell(start, 0).merge(t.cell(end, 0))
                except Exception:
                    pass
                r = end + 1
            else:
                r += 1
    # 表头跨页重复 + 行内不断页
    try:
        set_repeat_header(t.rows[0])
        for row in t.rows:
            set_cant_split(row)
    except Exception:
        pass
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def add_quote(doc, lines):
    for ln in lines:
        p = doc.add_paragraph()
        pf = p.paragraph_format
        pf.left_indent = Cm(0.8)
        pf.space_after = Pt(2)
        pf.line_spacing = 1.3
        add_inline(p, ln, 10, color=RGBColor(0x44, 0x44, 0x44))


def add_list(doc, text, ordered):
    style = "List Number" if ordered else "List Bullet"
    p = doc.add_paragraph(style=style)
    p.paragraph_format.line_spacing = 1.4
    p.paragraph_format.space_after = Pt(1)
    add_inline(p, text, 10.5)
    return p


def add_rule(doc):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(6)
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    bot = OxmlElement("w:bottom")
    bot.set(qn("w:val"), "single")
    bot.set(qn("w:sz"), "6")
    bot.set(qn("w:color"), "BBBBBB")
    pbdr.append(bot)
    pPr.append(pbdr)


def add_footer_pagenum(doc):
    sec = doc.sections[0]
    p = sec.footer.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run("第 ")
    set_run_font(r, BODY_FONT_CN, BODY_FONT_EN, 9, color=RGBColor(0x66, 0x66, 0x66))
    fld = OxmlElement("w:fldSimple")
    fld.set(qn("w:instr"), "PAGE")
    run_in = OxmlElement("w:r")
    t = OxmlElement("w:t")
    t.text = "1"
    run_in.append(t)
    fld.append(run_in)
    p._p.append(fld)
    r2 = p.add_run(" 页")
    set_run_font(r2, BODY_FONT_CN, BODY_FONT_EN, 9, color=RGBColor(0x66, 0x66, 0x66))


def convert(md_path, docx_path):
    src = io.open(md_path, encoding="utf-8").read().splitlines()

    doc = Document()
    sec = doc.sections[0]
    sec.page_width = Cm(21.0)
    sec.page_height = Cm(29.7)
    sec.top_margin = Cm(2.4)
    sec.bottom_margin = Cm(2.2)
    sec.left_margin = Cm(2.6)
    sec.right_margin = Cm(2.6)

    st = doc.styles["Normal"]
    st.font.name = BODY_FONT_EN
    st.font.size = Pt(10.5)
    st._element.rPr.rFonts.set(qn("w:eastAsia"), BODY_FONT_CN)

    i = 0
    n = len(src)
    in_code = False
    code_buf = []
    quote_buf = []
    title_done = False

    def flush_quote():
        if quote_buf:
            add_quote(doc, quote_buf)
            quote_buf.clear()

    while i < n:
        raw = src[i]
        line = raw.rstrip()

        # ---------- 代码块 ----------
        if line.startswith("```"):
            if in_code:
                add_code(doc, code_buf)
                code_buf = []
                in_code = False
            else:
                flush_quote()
                in_code = True
            i += 1
            continue
        if in_code:
            code_buf.append(line)
            i += 1
            continue

        # ---------- 空行 ----------
        if not line.strip():
            flush_quote()
            i += 1
            continue

        # ---------- 引用 ----------
        if line.startswith(">"):
            quote_buf.append(line.lstrip(">").strip())
            i += 1
            continue
        flush_quote()

        # ---------- 分隔线 ----------
        if re.match(r"^-{3,}$", line.strip()):
            add_rule(doc)
            i += 1
            continue

        # ---------- 标题 ----------
        m = re.match(r"^(#{1,4})\s+(.*)$", line)
        if m:
            lvl = len(m.group(1))
            txt = m.group(2).strip()
            if lvl == 1 and not title_done:
                add_heading(doc, txt, 1)
                title_done = True
            else:
                add_heading(doc, txt, min(lvl + 1, 4) if lvl >= 2 else 2)
            i += 1
            continue

        # ---------- 表格 ----------
        if line.lstrip().startswith("|"):
            rows = []
            while i < n and src[i].lstrip().startswith("|"):
                ln = src[i].strip()
                if re.match(r"^\|[\s:\-|]+\|$", ln):
                    i += 1
                    continue
                cells = split_row(ln)
                rows.append(cells)
                i += 1
            if rows:
                add_table(doc, rows)
            continue

        # ---------- 列表 ----------
        m = re.match(r"^(\s*)[-*]\s+(.*)$", line)
        if m:
            add_list(doc, m.group(2), False)
            i += 1
            continue
        m = re.match(r"^(\s*)(\d+)\.\s+(.*)$", line)
        if m:
            add_list(doc, m.group(3), True)
            i += 1
            continue

        # ---------- 缩进续行（属于上一项）----------
        if line.startswith("  ") and line.strip():
            add_body(doc, line.strip(), indent=False)
            i += 1
            continue

        # ---------- 普通段落 ----------
        add_body(doc, line.strip())
        i += 1

    flush_quote()
    if in_code and code_buf:
        add_code(doc, code_buf)

    add_footer_pagenum(doc)
    doc.save(docx_path)
    return doc


if __name__ == "__main__":
    md, out = sys.argv[1], sys.argv[2]
    d = convert(md, out)
    print("段落 %d 个, 表格 %d 个" % (len(d.paragraphs), len(d.tables)))
    print("已生成: " + out)
