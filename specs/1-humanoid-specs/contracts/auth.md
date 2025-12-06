# API Contracts: Authentication

This document defines the API endpoints for user authentication and authorization for the frontend application, to be implemented using FastAPI.

## 1. User Registration

-   **Endpoint**: `POST /auth/register`
-   **Description**: Registers a new user.
-   **Request Body**:
    -   `username` (string, required): Desired username.
    -   `email` (string, required): User's email address.
    -   `password` (string, required): User's chosen password.
-   **Response**:
    -   `201 Created`: User successfully registered.
        -   `message`: "User registered successfully"
    -   `400 Bad Request`: Invalid input (e.g., username/email already exists, weak password).
        -   `detail`: "Username already registered" or "Email already registered" or "Password is too weak"

## 2. User Login

-   **Endpoint**: `POST /auth/login`
-   **Description**: Authenticates a user and returns an access token.
-   **Request Body**:
    -   `username` (string, required): User's username.
    -   `password` (string, required): User's password.
-   **Response**:
    -   `200 OK`: User successfully authenticated.
        -   `access_token`: (string) JWT access token.
        -   `token_type`: "bearer"
    -   `401 Unauthorized`: Invalid credentials.
        -   `detail`: "Incorrect username or password"

## 3. Token Refresh (Optional)

-   **Endpoint**: `POST /auth/refresh`
-   **Description**: Refreshes an expired access token using a refresh token.
-   **Request Headers**:
    -   `Authorization`: `Bearer <refresh_token>`
-   **Response**:
    -   `200 OK`: Access token successfully refreshed.
        -   `access_token`: (string) New JWT access token.
        -   `token_type`: "bearer"
    -   `401 Unauthorized`: Invalid or expired refresh token.
        -   `detail`: "Invalid or expired refresh token"

## 4. User Profile

-   **Endpoint**: `GET /users/me`
-   **Description**: Retrieves the authenticated user's profile information.
-   **Request Headers**:
    -   `Authorization`: `Bearer <access_token>`
-   **Response**:
    -   `200 OK`: User profile data.
        -   `id` (UUID)
        -   `username` (string)
        -   `email` (string)
        -   `language_preference` (string)
        -   `theme_preference` (string)
        -   `is_active` (boolean)
        -   `is_admin` (boolean)
    -   `401 Unauthorized`: Invalid or missing access token.

## 5. Update User Preferences

-   **Endpoint**: `PUT /users/me/preferences`
-   **Description**: Updates the authenticated user's preferences (e.g., language, theme).
-   **Request Headers**:
    -   `Authorization`: `Bearer <access_token>`
-   **Request Body**:
    -   `language_preference` (string, optional): New language preference.
    -   `theme_preference` (string, optional): New theme preference.
-   **Response**:
    -   `200 OK`: Preferences updated.
        -   `message`: "Preferences updated successfully"
    -   `400 Bad Request`: Invalid input.
    -   `401 Unauthorized`: Invalid or missing access token.
