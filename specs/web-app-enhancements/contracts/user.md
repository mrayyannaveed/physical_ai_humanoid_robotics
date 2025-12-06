# User Profile API Contracts

## GET /user/profile

Retrieves the authenticated user's profile information.

-   **Description**: Fetches the profile details for the currently authenticated user.
-   **Authentication**: Requires valid JWT.
-   **Responses**:
    -   `200 OK`: User profile successfully retrieved.
        ```json
        {
            "id": "uuid",
            "user_id": "uuid",
            "preferred_language": "en",
            "theme": "dark",
            "first_name": "string",
            "last_name": "string",
            "bio": "string"
        }
        ```
    -   `401 Unauthorized`: No valid JWT provided.
        ```json
        {
            "detail": "Not authenticated"
        }
        ```
    -   `404 Not Found`: User profile not found (should ideally not happen if user is authenticated).
        ```json
        {
            "detail": "User profile not found"
        }
        ```

## PUT /user/profile

Updates the authenticated user's profile information.

-   **Description**: Allows the authenticated user to update their profile details.
-   **Authentication**: Requires valid JWT.
-   **Request Body**: `application/json`
    ```json
    {
        "preferred_language": "ur",
        "theme": "light",
        "first_name": "NewFirstName",
        "last_name": "NewLastName",
        "bio": "Updated biography text"
    }
    ```
    (All fields are optional; only provided fields will be updated)
-   **Responses**:
    -   `200 OK`: User profile successfully updated.
        ```json
        {
            "message": "Profile updated successfully",
            "profile_id": "uuid"
        }
        ```
    -   `400 Bad Request`: Invalid input (e.g., unsupported language/theme).
        ```json
        {
            "detail": "Validation error message"
        }
        ```
    -   `401 Unauthorized`: No valid JWT provided.
        ```json
        {
            "detail": "Not authenticated"
        }
        ```
    -   `404 Not Found`: User profile not found.
        ```json
        {
            "detail": "User profile not found"
        }
        ```