#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Function to encode a string to hexadecimal
void stringToHex(const char* input, char* file, int length) {
    for (int i = 0; i < length; i++) {
        sprintf(file + 2 * i, "%02X", input[i]);
    }
}

int main() {
    FILE *file;
    file = fopen("htmldata.c", "w");
    const char *folder[] = {
        //Add your file path here
        "./html_files/index.shtml"
    }; 
    const char *files[] = {
        "./index.shtml"
    };
    char *varnames[100] = {

    };
    
    int fileCount = sizeof(files)/sizeof(files[0]);
    for(int i = 0; i< fileCount ;i++){
        char header[1024];
        char fvar[256];
        if (strstr(files[i], "404")) {
            snprintf(header, sizeof(header), "HTTP/1.0 404 File not found\r\n");
        } else {
            snprintf(header, sizeof(header), "HTTP/1.0 200 OK\r\n");
        }

        snprintf(header + strlen(header), sizeof(header) - strlen(header),
            "Server: lwIP/pre-0.6 (http://www.sics.se/~adam/lwip/)\r\n");

        // Add content-type based on files extension
        if (strstr(files[i], ".html") || strstr(files[i], ".shtml")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: text/html\r\n");
        } else if (strstr(files[i], ".jpg")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: image/jpeg\r\n");
        } else if (strstr(files[i], ".gif")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: image/gif\r\n");
        } else if (strstr(files[i], ".png")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: image/png\r\n");
        } else if (strstr(files[i], ".class")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: application/octet-stream\r\n");
        } else if (strstr(files[i], ".js")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: text/javascript\r\n");
        } else if (strstr(files[i], ".css")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: text/css\r\n");
        } else if (strstr(files[i], ".svg")) {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: image/svg+xml\r\n");
        } else {
            snprintf(header + strlen(header), sizeof(header) - strlen(header), "Content-type: text/plain\r\n");
        }

        snprintf(header + strlen(header), sizeof(header) - strlen(header), "\r\n");

        // Create a variable name for the files[i]
        strcpy(fvar, files[i] + 1);  // Remove the leading dot in the filename
        for (int j = 0; fvar[j]; j++) {
            if (fvar[j] == '/' || fvar[j] == '\\') {
                fvar[j] = '_';
            } else if (fvar[j] == '.') {
                fvar[j] = '_';
            }
        }

        fprintf(file, "static const unsigned char data%s[] = {\n", fvar);
        fprintf(file, "\t/* %s */\n\t", files[i]);

        // Encode the filename as hexadecimal
        char hexFileName[2 * strlen(files[i]) + 1];
        stringToHex(files[i] + 1, hexFileName, strlen(files[i]) - 1);
        int count = 0;
        for (int j = 0; hexFileName[j]; j++) {
            fprintf(file, "0x%c%c, ", hexFileName[j], hexFileName[j + 1]);
            j++;
            count++;
        }
        fprintf(file, "0x00,\n\t");

        // Encode the HTTP header as hexadecimal
        char hexHeader[2 * strlen(header) + 1];
        stringToHex(header, hexHeader, strlen(header));
        count = 0;
        for (int j = 0; hexHeader[j]; j++) {
            fprintf(file, "0x%c%c, ", hexHeader[j], hexHeader[j + 1]);
            j++;
            count++;
            if (count == 10) {
                fprintf(file, "\n\t");
                count = 0;
            }
        }
        fprintf(file, "\n\t");
         // Encode the file content as hexadecimal
        FILE* filePtr = fopen(folder[i], "rb");
        count = 0;
        if (filePtr) {
            int ch;
            while ((ch = fgetc(filePtr)) != EOF) {
                fprintf(file, "0x%02X, ", ch);
                count++;
                if (count == 10) {
                    fprintf(file, "\n\t");
                    count = 0;
                }
            }
            fclose(filePtr);
        }

        fprintf(file, "};\n\n");
        varnames[i] = malloc(strlen(fvar) + 1);
        strcpy(varnames[i], fvar); 
        
        
        fprintf(file, "const struct fsdata_file file%s[] = {{ %s, data%s, data%s + %d, sizeof(data%s) - %d, FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT }};\n",
            fvar,"NULL", fvar, fvar, strlen(files[i]) , fvar, strlen(files[i]));
    }

    fprintf(file, "\n#define FS_ROOT file%s\n",varnames[fileCount - 1] );
    fprintf(file, "#define FS_NUMFILES %d\n", fileCount);

    fclose(file);
}
