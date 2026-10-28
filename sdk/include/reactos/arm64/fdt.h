/*
 * PROJECT:     LiberNT ARM64 support
 * LICENSE:     GPL-3.0-or-later (https://spdx.org/licenses/GPL-3.0-or-later)
 * PURPOSE:     Minimal read-only flattened devicetree lookup shared by the loader and the HAL
 * COPYRIGHT:   Copyright 2026 Ahmed ARIF <arif.ing@outlook.com>
 */

#pragma once

#define ARM64_FDT_MAGIC             0xD00DFEEDU
#define ARM64_FDT_HEADER_SIZE       40U
#define ARM64_FDT_BEGIN_NODE        1U
#define ARM64_FDT_END_NODE          2U
#define ARM64_FDT_PROP              3U
#define ARM64_FDT_NOP               4U
#define ARM64_FDT_MAX_DEPTH         16U

static __inline UINT32
Arm64FdtRead32(_In_ const UCHAR *Data)
{
    return ((UINT32)Data[0] << 24) | ((UINT32)Data[1] << 16) |
           ((UINT32)Data[2] << 8) | (UINT32)Data[3];
}

static __inline BOOLEAN
Arm64FdtGetSize(
    _In_ const VOID *Fdt,
    _Out_ PULONG TotalSize)
{
    const UCHAR *Header = Fdt;
    UINT32 Total;

    *TotalSize = 0;
    if (!Header || Arm64FdtRead32(Header) != ARM64_FDT_MAGIC)
        return FALSE;

    Total = Arm64FdtRead32(Header + 4);
    if (Total < ARM64_FDT_HEADER_SIZE)
        return FALSE;

    *TotalSize = Total;
    return TRUE;
}

static __inline BOOLEAN
Arm64FdtStringIs(
    _In_ const UCHAR *Text,
    _In_ const UCHAR *Limit,
    _In_ PCSTR Wanted)
{
    while (*Wanted)
    {
        if (Text >= Limit || *Text != (UCHAR)*Wanted)
            return FALSE;

        Text++;
        Wanted++;
    }

    return Text < Limit && *Text == 0;
}

static __inline BOOLEAN
Arm64FdtListHas(
    _In_reads_bytes_(Length) const UCHAR *Data,
    _In_ UINT32 Length,
    _In_ PCSTR Wanted)
{
    UINT32 Offset = 0;
    UINT32 Start;

    while (Offset < Length)
    {
        Start = Offset;
        while (Offset < Length && Data[Offset])
            Offset++;

        if (Offset >= Length)
            break;

        if (Arm64FdtStringIs(Data + Start, Data + Length, Wanted))
            return TRUE;

        Offset++;
    }

    return FALSE;
}

static __inline UINT64
Arm64FdtReadCells(
    _In_ const UCHAR *Data,
    _In_ UINT32 Cells)
{
    UINT64 Value = Arm64FdtRead32(Data);

    if (Cells == 2)
        Value = (Value << 32) | Arm64FdtRead32(Data + 4);

    return Value;
}

static __inline BOOLEAN
Arm64FdtFindReg(
    _In_ const VOID *FdtBlob,
    _In_ PCSTR Compatible,
    _In_ ULONG RegIndex,
    _Out_opt_ PULONGLONG Address,
    _Out_opt_ PULONGLONG Size)
{
    const UCHAR *Fdt = FdtBlob;
    const UCHAR *Cursor;
    const UCHAR *StructEnd;
    const UCHAR *Strings;
    const UCHAR *StringsEnd;
    const UCHAR *Reg[ARM64_FDT_MAX_DEPTH];
    UINT32 RegLength[ARM64_FDT_MAX_DEPTH];
    UINT32 AddressCells[ARM64_FDT_MAX_DEPTH];
    UINT32 SizeCells[ARM64_FDT_MAX_DEPTH];
    BOOLEAN Match[ARM64_FDT_MAX_DEPTH];
    BOOLEAN Enabled[ARM64_FDT_MAX_DEPTH];
    UINT32 Total, StructOffset, StructSize, StringsOffset, StringsSize;
    UINT32 Token, Length, NameOffset, Ac, Sc, Stride;
    LONG Depth = -1;

    if (Address)
        *Address = 0;
    if (Size)
        *Size = 0;

    if (!Fdt || Arm64FdtRead32(Fdt) != ARM64_FDT_MAGIC)
        return FALSE;

    Total = Arm64FdtRead32(Fdt + 4);
    StructOffset = Arm64FdtRead32(Fdt + 8);
    StringsOffset = Arm64FdtRead32(Fdt + 12);
    StringsSize = Arm64FdtRead32(Fdt + 32);
    StructSize = Arm64FdtRead32(Fdt + 36);

    if (Total < ARM64_FDT_HEADER_SIZE ||
        StructOffset > Total || StructSize > Total - StructOffset ||
        StringsOffset > Total || StringsSize > Total - StringsOffset)
    {
        return FALSE;
    }

    Cursor = Fdt + StructOffset;
    StructEnd = Cursor + StructSize;
    Strings = Fdt + StringsOffset;
    StringsEnd = Strings + StringsSize;

    while (Cursor + 4 <= StructEnd)
    {
        Token = Arm64FdtRead32(Cursor);
        Cursor += 4;

        if (Token == ARM64_FDT_BEGIN_NODE)
        {
            while (Cursor < StructEnd && *Cursor)
                Cursor++;

            if (Cursor >= StructEnd)
                return FALSE;

            Cursor = Fdt + ((((ULONG_PTR)(Cursor + 1 - Fdt)) + 3) & ~(ULONG_PTR)3);
            if (++Depth >= (LONG)ARM64_FDT_MAX_DEPTH)
                return FALSE;

            AddressCells[Depth] = 2;
            SizeCells[Depth] = 1;
            Match[Depth] = FALSE;
            Enabled[Depth] = TRUE;
            Reg[Depth] = NULL;
            RegLength[Depth] = 0;
        }
        else if (Token == ARM64_FDT_END_NODE)
        {
            if (Depth < 0)
                return FALSE;

            Ac = Depth > 0 ? AddressCells[Depth - 1] : 2;
            Sc = Depth > 0 ? SizeCells[Depth - 1] : 1;
            Stride = (Ac + Sc) * 4;
            if (Match[Depth] && Enabled[Depth] && Reg[Depth] &&
                (Ac == 1 || Ac == 2) && (Sc <= 2) &&
                RegLength[Depth] >= (RegIndex + 1) * Stride)
            {
                const UCHAR *Entry = Reg[Depth] + RegIndex * Stride;

                if (Address)
                    *Address = Arm64FdtReadCells(Entry, Ac);
                if (Size)
                    *Size = Sc ? Arm64FdtReadCells(Entry + Ac * 4, Sc) : 0;

                return TRUE;
            }

            Depth--;
        }
        else if (Token == ARM64_FDT_PROP)
        {
            const UCHAR *Data;
            const UCHAR *Name;

            if (StructEnd - Cursor < 8)
                return FALSE;

            Length = Arm64FdtRead32(Cursor);
            NameOffset = Arm64FdtRead32(Cursor + 4);
            Cursor += 8;

            if (Length > (UINT32)(StructEnd - Cursor) || NameOffset >= StringsSize)
                return FALSE;

            Data = Cursor;
            Name = Strings + NameOffset;
            Cursor = Fdt + ((((ULONG_PTR)(Cursor + Length - Fdt)) + 3) & ~(ULONG_PTR)3);

            if (Depth < 0)
                continue;

            if (Arm64FdtStringIs(Name, StringsEnd, "#address-cells") && Length == 4)
                AddressCells[Depth] = Arm64FdtRead32(Data);
            else if (Arm64FdtStringIs(Name, StringsEnd, "#size-cells") && Length == 4)
                SizeCells[Depth] = Arm64FdtRead32(Data);
            else if (Arm64FdtStringIs(Name, StringsEnd, "compatible"))
                Match[Depth] = Arm64FdtListHas(Data, Length, Compatible);
            else if (Arm64FdtStringIs(Name, StringsEnd, "status"))
                Enabled[Depth] = Length >= 2 && Data[0] == 'o' && Data[1] == 'k';
            else if (Arm64FdtStringIs(Name, StringsEnd, "reg"))
            {
                Reg[Depth] = Data;
                RegLength[Depth] = Length;
            }
        }
        else if (Token != ARM64_FDT_NOP)
        {
            break;
        }
    }

    return FALSE;
}
