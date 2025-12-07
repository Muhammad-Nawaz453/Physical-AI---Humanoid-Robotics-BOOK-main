import React, { useEffect } from 'react';
import RAGChatbot from '../components/RAGChatbot';
import API_URL from '../config';
export default function Root({ children }) {
    useEffect(() => {
        // Check if user is logged in
        const user = localStorage.getItem('user');
        const currentPath = window.location.pathname;



        // Allow access only to signup/login pages without authentication
        const publicPaths = ['/signup', '/login'];
        const isPublicPath = publicPaths.includes(currentPath);

        if (!user && !isPublicPath) {
            // Redirect to login if not authenticated
            window.location.href = '/login';
        }
    }, []);

    return (
        <>
            {children}
            <RAGChatbot />
        </>
    );
}