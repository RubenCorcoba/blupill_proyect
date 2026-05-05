#include <stdint.h>
#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define PUERTO "4000"

int main(int argc, char **argv) {
    struct addrinfo *result = NULL;
    static struct addrinfo hints;
    int iResult;
    static WSADATA wsaData;
    int numreads = -1;

    if (argc > 1) {
        numreads = atoi(argv[2]);
        if (!numreads) numreads = -1;
    }

    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if (iResult) {
        fprintf(stderr, "Falla %d en WSAStartup\n", iResult);
        return 1;
    }

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    iResult = getaddrinfo(NULL, PUERTO, &hints, &result);

    if (iResult) {
        fprintf(stderr, "Falla %d en getaddrinfo\n", iResult);
        WSACleanup();
        return 1;
    }

    SOCKET listenSocket = INVALID_SOCKET, clientSocket = INVALID_SOCKET;

    listenSocket =
        socket(result->ai_family, result->ai_socktype, result->ai_protocol);

    if (listenSocket == INVALID_SOCKET) {
        fprintf(stderr, "Error %d en socket()\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    iResult = bind(listenSocket, result->ai_addr, (int)result->ai_addrlen);

    if (iResult == SOCKET_ERROR) {
        fprintf(stderr, "Error %d en bind()\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }
    freeaddrinfo(result);
    if (listen(listenSocket, 2) == SOCKET_ERROR) {
        fprintf(stderr, "Error %d en listen()\n", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET) {
        fprintf(stderr, "Error %d en accept()\n", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    closesocket(listenSocket);

    do {
        static char recvbuf[1024];
        iResult = recv(clientSocket, recvbuf, sizeof(recvbuf), 0);
        if (iResult > 0) {
            struct Sample_s {
                int16_t i, q;
            } *recvsmp = (struct Sample_s *)recvbuf;

            const int nsamples = iResult / sizeof(struct Sample_s);

            for (int i = 0; i < nsamples; ++i) {
                printf("%hd%+hdj\n", recvsmp[i].i, recvsmp[i].q);
            }
            if (numreads > 0) {
                numreads = (numreads < nsamples) ? 0 : numreads - nsamples;
            }
        } else if (iResult < 0) {
            fprintf(stderr, "Error %d en recv\n", WSAGetLastError());
            closesocket(clientSocket);
            WSACleanup();
            return 1;
        }
    } while (iResult && numreads);
    iResult = shutdown(clientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        fprintf(stderr, "Error %d en shutdown\n", WSAGetLastError());
        closesocket(clientSocket);
        WSACleanup();
        return 1;
    }
    closesocket(clientSocket);
    WSACleanup();
    return 0;
}
