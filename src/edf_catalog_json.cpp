#include "edf_catalog.h"

#include <stdio.h>
#include <time.h>

namespace EdfCatalog {

bool PageCursor::next(const Page &page) {
    if (done_) return false;
    if (!entries_) {
        entries_ = true;
        token("{\"sessions\":[");
        return true;
    }
    if (row_ == page.count) {
        switch (field_++) {
            case 0:
                if (!page.str.present) {
                    field_ = 5;
                    token("],\"str\":null");
                } else token("],\"str\":{\"path\":\"/STR.edf\"");
                return true;
            case 1:
                snprintf(number_, sizeof(number_), "%llu", (unsigned long long)page.str.size);
                field(",\"size\":", number_, false);
                return true;
            case 2:
                if (page.str.modified > 0) integer(",\"modified\":", page.str.modified);
                else token(",\"modified\":null");
                return true;
            case 3: field(",\"etag\":", page.str.etag); return true;
            case 4: token("}"); return true;
            case 5:
                if (page.next_cursor[0]) field(",\"next_cursor\":", page.next_cursor);
                else token(",\"next_cursor\":null");
                return true;
            default:
                done_ = true;
                token("}");
                return true;
        }
    }
    const SessionMetadata &entry = page.sessions[row_];
    if (files_) {
        while (file_ < SESSION_FILE_COUNT && !(entry.files_mask & (1u << file_))) ++file_;
        if (file_ == SESSION_FILE_COUNT) {
            files_ = false;
            field_ = 0;
            ++row_;
            token("]}");
            return true;
        }
        switch (field_++) {
            case 0: {
                snprintf(text_, sizeof(text_), "/DATALOG/%s/%s_%s.edf%s",
                         entry.therapy_day, entry.file_prefix, SESSION_SUFFIXES[file_],
                         entry.part_mask & (1u << file_) ? ".part" : "");
                field(entry.files_mask & ((1u << file_) - 1) ? ",{\"path\":" : "{\"path\":", text_);
                return true;
            }
            case 1: integer(",\"size\":", entry.file_sizes[file_]); return true;
            default:
                field_ = 0;
                ++file_;
                token("}");
                return true;
        }
    }
    switch (field_++) {
        case 0: field(row_ ? ",{\"day\":" : "{\"day\":", entry.therapy_day); return true;
        case 1:
        case 2: {
            const bool end = field_ == 3;
            if (end && (!entry.closed || entry.end_seconds <= 0)) token(",\"end\":null");
            else {
                // These seconds encode the EDF civil calendar, not a UTC instant.
                const time_t seconds = end ? entry.end_seconds : entry.start_seconds;
                struct tm civil = {};
                gmtime_r(&seconds, &civil);
                strftime(text_, sizeof(text_), "%Y-%m-%dT%H:%M:%S", &civil);
                field(end ? ",\"end\":" : ",\"start\":", text_);
            }
            return true;
        }
        case 3:
            field(",\"closed\":", entry.closed ? "true" : "false", false);
            return true;
        default:
            field_ = 0;
            file_ = 0;
            files_ = true;
            token(",\"files\":[");
            return true;
    }
}

size_t PageCursor::read(const Page &page, char *out, size_t capacity) {
    return read_tokens(out, capacity, [&] { return next(page); });
}

}  // namespace EdfCatalog
