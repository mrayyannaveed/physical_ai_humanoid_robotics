# Authentication API Contracts

## POST /auth/signup

Registers a new user.

-   **Description**: Allows a new user to create an account by providing a username, email, and password.
-   **Request Body**: `application/json`
    ```json
    {
        "username": "string",
        "email": "user@example.com",
        "password": "strongpassword123"
    }
    ```
-   **Responses**:
    -   `201 Created`: User successfully registered.
        ```json
        {
            "message": "User registered successfully",
            "user_id": "uuid"
        }
        ```
    -   `400 Bad Request`: Invalid input (e.g., weak password, invalid email format).
        ```json
        {
            "detail": "Validation error message"
        }
        ```
    -   `409 Conflict`: Username or email already exists.
        ```json
        {
            "detail": "Username or email already registered"
        }
        ```

## POST /auth/signin

Authenticates a user and returns a JWT.

-   **Description**: Authenticates an existing user with username/email and password, returning an access token upon success.
-   **Request Body**: `application/json`
    ```json
    {
        "username_or_email": "string",
        "password": "strongpassword123"
    }
    ```
-   **Responses**:
    -   `200 OK`: User authenticated, JWT returned in HttpOnly cookie.
        ```json
        {
            "message": "Login successful",
            "access_token": "jwt_token_string",
            "token_type": "bearer"
        }
        ```
    -   `401 Unauthorized`: Invalid credentials.
        ```json
        {
            "detail": "Incorrect username or password"
        }
        ```