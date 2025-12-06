# Quickstart Guide for Web Application Development

This guide provides a quick overview of how to set up and run the web application development environment, including both the React frontend and FastAPI backend.

## Prerequisites

-   Python 3.10+
-   Node.js (LTS recommended) and npm/yarn
-   Docker (optional, for database setup)
-   Git

## 1. Backend Setup (FastAPI)

Navigate to the `backend/` directory from the project root.

```bash
cd backend
```

### 1.1. Create and Activate Virtual Environment

```bash
python -m venv venv
# On Windows
./venv/Scripts/activate
# On macOS/Linux
source venv/bin/activate
```

### 1.2. Install Dependencies

```bash
pip install -r requirements.txt # (assuming requirements.txt will be generated)
# Or, if using poetry/pipenv:
# poetry install
# pipenv install
```

### 1.3. Database Setup

For local development, you can use SQLite or set up a PostgreSQL instance via Docker.

**Using SQLite (for simple local dev):**

The application will likely use a `database.db` file in the `backend/` directory by default. No extra setup required beyond ensuring the ORM (e.g., SQLAlchemy) can create tables.

**Using PostgreSQL with Docker:**

```bash
docker run --name some-postgres -e POSTGRES_PASSWORD=mysecretpassword -p 5432:5432 -d postgres
```

Update your `.env` file in the `backend/` directory with the PostgreSQL connection string.

### 1.4. Run Database Migrations

```bash
# Assuming Alembic for migrations
alembic upgrade head
```

### 1.5. Run the Backend Server

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The backend API will be accessible at `http://localhost:8000`.

## 2. Frontend Setup (React)

Navigate to the `frontend/` directory from the project root.

```bash
cd frontend
```

### 2.1. Install Dependencies

```bash
npm install
# Or
yarn install
```

### 2.2. Run the Frontend Development Server

```bash
npm start
# Or
yarn start
```

The React development server will typically start at `http://localhost:3000`. It will proxy API requests to the backend if configured correctly (e.g., in `package.json` or `setupProxy.js`).

## 3. Accessing the Application

-   Open your web browser and navigate to `http://localhost:3000` to access the frontend.
-   The backend API documentation (Swagger UI) will be available at `http://localhost:8000/docs`.

## Important Notes

-   Ensure your `.env` files (for both frontend and backend) are correctly configured with necessary environment variables (e.g., database URLs, JWT secrets, API endpoints).
-   Refer to the `README.md` in both `frontend/` and `backend/` for more detailed, framework-specific instructions.
